//! Stage 3 — cluster merging (Fudos–Hoffmann, generalised) → plan → replay.
//!
//! Decomposition (once per topology):
//!   * every PP/PL edge seeds a 2-element rigid cluster; ground (fixed points + x-axis) is a fixed
//!     cluster; lines carry direction classes (union-find with angular offsets);
//!   * repeatedly merge a pair or a triple of clusters when what they share — points, lines,
//!     directions — determines their relative rigid transforms.  F–H's triangle rule is the common
//!     case; the decision is made generally by the rank of the small merge Jacobian at generic
//!     (witness) poses, with self-motions of degenerate clusters accounted for, so parallels,
//!     perpendiculars and H/V need no special cases;
//!   * when pair/triple merging stalls, look for a small core: a minimal subset of clusters that is
//!     rigid as a whole (Stage 3b), merge it as one numeric step, resume tree merging;
//!   * the merge sequence is the plan; the clusters left over are the roots.
//!
//! Execution (every solve / drag frame, no graph analysis):
//!   * leaf poses from the live dimension values, warm-started on the current geometry;
//!   * PPP triangle merges by ruler-and-compass with an explicit chirality flag; other merges by a
//!     small minimum-norm Newton (DogLeg if it does not converge);
//!   * unfixed roots placed by least-change (Procrustes onto current positions);
//!   * write back; verify with the compiled System; numeric fallback if needed.

use crate::cgraph::{
    build, line_normal, normal_of, remainder, ConstraintGraph, Edge, EdgeKind, El, ElKind, X_AXIS,
};
use crate::linalg::{absmax, min_norm_solve, rank_rrqr, Mat};
use crate::io::Part;
use crate::model::{increments, orientation, EntRef, Sketch};
use crate::newton::{self, Method, Tol, TrustRegion};
use crate::rng::Rng;
use crate::solve::{Drag, SolveOpts, SolveResult, Triangle};
use crate::system::System;
use std::mem::take;
use std::collections::{BTreeMap, BTreeSet};

/// point: (x, y); line: (nx, ny, c)
pub type Pose = Vec<f64>;

fn x_pose() -> Pose {
    vec![0.0, 1.0, 0.0]
}

pub type Pair = (usize, usize, El);
pub type DPair = (usize, usize, El, El, f64);

#[derive(Clone, Debug)]
pub struct Cluster {
    pub id: usize,
    pub els: BTreeMap<El, Pose>,
    pub fixed: bool,
}

/// One merge, lowered for replay.  `ids[0]` is the reference cluster (identity transform);
/// pairs/dpairs use positions into `ids`; a PPP triangle carries its (x, y, z) construction and
/// `branch` (±1 chirality: the orientation of (x, z, y)) — set by the replay from the sketch when
/// `None`, so a persisted plan replays the recorded root.
#[derive(Clone, Debug)]
pub struct Step {
    pub ids: Vec<usize>,
    pub pairs: Vec<Pair>,
    pub dpairs: Vec<DPair>,
    pub ppp: Option<(El, El, El)>,
    pub branch: Option<i32>,
}

impl Step {
    pub fn out(&self) -> usize {
        self.ids[0]
    }

    /// Document-stable identity of a closed-form construction (`None` for numeric merges).  Keyed
    /// by the sketch indices of the three points, not by compiled element indices, so it survives
    /// save/load and edits that renumber elements.
    pub fn key(&self, g: &ConstraintGraph) -> Option<String> {
        self.ppp
            .map(|(a, b, c)| branch_key([g.point_index(a), g.point_index(b), g.point_index(c)]))
    }
}

/// A recorded root choice is keyed by the three sketch points of its closed-form construction —
/// document-stable identity, so a choice survives a recompile.  It names points, so anything that
/// renumbers points has to carry the key with them: see `branch_key_points`.
pub fn branch_key(pts: [usize; 3]) -> String {
    format!("ppp:{}|{}|{}", pts[0], pts[1], pts[2])
}

/// The three sketch points a branch key names, or `None` if the key is not one we wrote.
pub fn branch_key_points(k: &str) -> Option<[usize; 3]> {
    let rest = k.strip_prefix("ppp:")?;
    let mut it = rest.split('|');
    let mut out = [0usize; 3];
    for slot in out.iter_mut() {
        *slot = it.next()?.parse().ok()?;
    }
    if it.next().is_some() {
        return None;
    }
    Some(out)
}

pub struct Plan {
    pub graph: ConstraintGraph,
    pub leaves: Vec<(usize, usize)>,
    pub ground_id: usize,
    pub singletons: Vec<(usize, El)>,
    pub steps: Vec<Step>,
    pub roots: Vec<usize>,
    /// True: replay the recorded chirality even if the sketch moved (Stage 5).
    pub sticky_branches: bool,
    /// What each root holds — the rigid bodies a drag moves (`Wave`).
    pub root_els: BTreeMap<usize, Vec<El>>,
    /// Per root, one line of each direction class it carries, by class root.
    pub root_dirs: BTreeMap<usize, BTreeMap<El, El>>,
    /// The roots holding each element.
    pub roots_of: BTreeMap<El, Vec<usize>>,
    /// Every non-point element's direction class: (class root, angle relative to it).
    pub droot: BTreeMap<El, (El, f64)>,
    /// The roots carrying a line of each direction class, by class root.
    pub class_roots: BTreeMap<El, Vec<usize>>,
}

impl Plan {
    /// What a drag can check the plan it is handed back against: cheap, and different for any
    /// plan of a different sketch or a different decomposition.  Identity would be better, but
    /// a plan crosses the ABI as a bare pointer, so shape is what there is.
    pub fn shape(&self) -> (usize, usize, usize) {
        (self.steps.len(), self.roots.len(), self.graph.members.len())
    }

    pub fn fully_decomposed(&self) -> bool {
        self.graph.unsupported.is_empty() && self.roots.len() == 1
    }

    /// Recorded root choices of the closed-form merges, keyed stably for persistence.
    pub fn branches(&self) -> BTreeMap<String, i32> {
        let mut out = BTreeMap::new();
        for st in &self.steps {
            if let (Some(k), Some(b)) = (st.key(&self.graph), st.branch) {
                out.insert(k, b);
            }
        }
        out
    }

    /// Install recorded root choices (e.g. from a document); returns how many matched.
    pub fn apply_branches(&mut self, branches: &BTreeMap<String, i32>) -> usize {
        let keys: Vec<Option<String>> = self.steps.iter().map(|s| s.key(&self.graph)).collect();
        let mut n = 0;
        for (i, k) in keys.into_iter().enumerate() {
            if let Some(k) = k {
                if let Some(&v) = branches.get(&k) {
                    self.steps[i].branch = Some(if v >= 0 { 1 } else { -1 });
                    n += 1;
                }
            }
        }
        n
    }

    /// (index, step) of every merge that places `e`: closed-form ones where `e` is the constructed
    /// apex, else numeric merges that share it.
    pub fn steps_placing(&self, e: El) -> Vec<usize> {
        let closed: Vec<usize> = self
            .steps
            .iter()
            .enumerate()
            .filter(|(_, st)| st.ppp.map(|p| p.1 == e).unwrap_or(false))
            .map(|(i, _)| i)
            .collect();
        if !closed.is_empty() {
            return closed;
        }
        self.steps
            .iter()
            .enumerate()
            .filter(|(_, st)| st.ppp.is_none() && st.pairs.iter().any(|&(_, _, x)| x == e))
            .map(|(i, _)| i)
            .collect()
    }

    /// Flip the root of every closed-form merge that constructs `e`; returns how many.
    pub fn flip(&mut self, e: El) -> usize {
        let idxs: Vec<usize> = self
            .steps_placing(e)
            .into_iter()
            .filter(|&i| self.steps[i].ppp.is_some())
            .collect();
        for &i in &idxs {
            self.steps[i].branch = Some(-self.steps[i].branch.unwrap_or(1));
        }
        idxs.len()
    }

    pub fn summary(&self) -> String {
        format!(
            "{} leaves, {} merges -> {} root(s); {} unsupported constraint(s)",
            self.leaves.len(),
            self.steps.len(),
            self.roots.len(),
            self.graph.unsupported.len()
        )
    }
}

/// How many entries repeat one already seen — the sum of `m-1` over the distinct values, which
/// is what a transitive relation held by `m` clusters is worth.  Sorts in place.
fn repeats(v: &mut [El]) -> usize {
    v.sort_unstable();
    v.windows(2).filter(|w| w[0] == w[1]).count()
}

/* -- direction classes: weighted union-find, potential = angle relative to the root ------- */

#[derive(Default)]
struct Dirs {
    parent: BTreeMap<El, El>,
    pot: BTreeMap<El, f64>,
}

impl Dirs {
    fn add(&mut self, e: El) {
        self.parent.entry(e).or_insert(e);
        self.pot.entry(e).or_insert(0.0);
    }

    /// (root, angle of `e` relative to the root).
    fn find(&mut self, e: El) -> (El, f64) {
        self.add(e);
        let mut path: Vec<El> = Vec::new();
        let mut cur = e;
        while self.parent[&cur] != cur {
            path.push(cur);
            cur = self.parent[&cur];
        }
        let root = cur;
        let mut acc = 0.0;
        for i in (0..path.len()).rev() {
            let x = path[i];
            acc += self.pot[&x];
            self.parent.insert(x, root);
            self.pot.insert(x, acc);
        }
        let p = if path.is_empty() { 0.0 } else { self.pot[&path[0]] };
        (root, p)
    }

    /// Impose n_b = rot(phi) n_a.  `false` if it contradicts an existing relation.
    fn join(&mut self, a: El, b: El, phi: f64) -> bool {
        let (ra, pa) = self.find(a);
        let (rb, pb) = self.find(b);
        if ra == rb {
            return remainder(pb - pa - phi, std::f64::consts::PI).abs() < 1e-9;
        }
        self.parent.insert(rb, ra);
        self.pot.insert(rb, pa + phi - pb);
        true
    }

    fn offset(&mut self, a: El, b: El) -> Option<f64> {
        let (ra, pa) = self.find(a);
        let (rb, pb) = self.find(b);
        if ra == rb {
            Some(pb - pa)
        } else {
            None
        }
    }
}

/* -- rigid transforms ------------------------------------------------------------------- */

/// Apply the transform T = (cos, sin, tx, ty) to an element's pose.
pub fn apply_t(t: &[f64; 4], e: El, pose: &[f64]) -> Pose {
    let (c, s, tx, ty) = (t[0], t[1], t[2], t[3]);
    if e.is_point() {
        return vec![c * pose[0] - s * pose[1] + tx, s * pose[0] + c * pose[1] + ty];
    }
    let nx2 = c * pose[0] - s * pose[1];
    let ny2 = s * pose[0] + c * pose[1];
    vec![nx2, ny2, pose[2] + nx2 * tx + ny2 * ty]
}

pub fn make_t(theta: f64, tx: f64, ty: f64) -> [f64; 4] {
    [theta.cos(), theta.sin(), tx, ty]
}

/// Pose of `e` under (theta, t) and its Jacobian with respect to (theta, tx, ty).
fn pose_jac(e: El, pose: &[f64], th: f64, tx: f64, ty: f64) -> Vec<[f64; 3]> {
    let (c, s) = (th.cos(), th.sin());
    if e.is_point() {
        let (x, y) = (pose[0], pose[1]);
        return vec![[-s * x - c * y, 1.0, 0.0], [c * x - s * y, 0.0, 1.0]];
    }
    let (nx, ny) = (pose[0], pose[1]);
    let n0 = c * nx - s * ny;
    let n1 = s * nx + c * ny;
    let d0 = -s * nx - c * ny;
    let d1 = c * nx - s * ny;
    vec![[d0, 0.0, 0.0], [d1, 0.0, 0.0], [d0 * tx + d1 * ty, n0, n1]]
}

/// Rigid transform (c, s, tx, ty) mapping points `src` onto `dst` in least squares.
fn procrustes(src: &[(f64, f64)], dst: &[(f64, f64)]) -> [f64; 4] {
    let n = src.len();
    if n == 0 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let (mut msx, mut msy, mut mdx, mut mdy) = (0.0, 0.0, 0.0, 0.0);
    for i in 0..n {
        msx += src[i].0;
        msy += src[i].1;
        mdx += dst[i].0;
        mdy += dst[i].1;
    }
    let f = n as f64;
    msx /= f;
    msy /= f;
    mdx /= f;
    mdy /= f;
    if n == 1 {
        return [1.0, 0.0, mdx - msx, mdy - msy];
    }
    let (mut c, mut s) = (0.0, 0.0);
    for i in 0..n {
        let (ax, ay) = (src[i].0 - msx, src[i].1 - msy);
        let (bx, by) = (dst[i].0 - mdx, dst[i].1 - mdy);
        c += ax * bx + ay * by;
        s += ax * by - ay * bx;
    }
    let l = {
        let h = c.hypot(s);
        if h == 0.0 {
            1.0
        } else {
            h
        }
    };
    c /= l;
    s /= l;
    [c, s, mdx - (c * msx - s * msy), mdy - (s * msx + c * msy)]
}

/// Rigid transform taking the segment p→q onto p2→q2 (exact when the lengths agree).
fn fit2(p: &[f64], q: &[f64], p2: &[f64], q2: &[f64]) -> [f64; 4] {
    let (ux, uy) = (q[0] - p[0], q[1] - p[1]);
    let (vx, vy) = (q2[0] - p2[0], q2[1] - p2[1]);
    let mut c = ux * vx + uy * vy;
    let mut s = ux * vy - uy * vx;
    let l = {
        let h = c.hypot(s);
        if h == 0.0 {
            1.0
        } else {
            h
        }
    };
    c /= l;
    s /= l;
    [c, s, p2[0] - (c * p[0] - s * p[1]), p2[1] - (s * p[0] + c * p[1])]
}

/* -- the merge system (shared by the generic-rank decision and by execution) -------------- */

/// A pose moved by (cx, cy): what `apply_t` sees when a transform is taken about a centre.
fn shift(e: El, pose: &[f64], cx: f64, cy: f64) -> Pose {
    if e.is_point() {
        vec![pose[0] + cx, pose[1] + cy]
    } else {
        vec![pose[0], pose[1], pose[2] + pose[0] * cx + pose[1] * cy]
    }
}

/// Residual/Jacobian for the transforms of `cl[1..]` (`cl[0]` is the reference, identity).
///
/// Each moving cluster's (θ, tx, ty) rotates it about its own `centre` (the origin unless set):
/// for the determined merges of a replay that is immaterial, but where the system is
/// under-determined and the minimum-norm step decides, it is what makes "least change" mean
/// the least rigid motion of *that body* — about the world origin a body far from it would
/// rather swing than slide, since a small θ moves it a long way for a small norm.
pub struct MergeSystem<'a> {
    pub cl: &'a [Cluster],
    pub pairs: &'a [Pair],
    pub dpairs: &'a [DPair],
    pub m: usize,
    pub n: usize,
    centres: Vec<[f64; 2]>,
    /// The rotation unknown of each cluster is an arc length: θ = u / radius.  At 1 it is the
    /// angle itself; at a body's radius of gyration a unit of it displaces the body as much as
    /// a unit of translation does, which is the norm "least change" ought to be measured in.
    radii: Vec<f64>,
}

impl<'a> MergeSystem<'a> {
    pub fn new(
        cl: &'a [Cluster],
        pairs: &'a [Pair],
        dpairs: &'a [DPair],
        k_movable: usize,
    ) -> MergeSystem<'a> {
        let m = pairs.iter().map(|&(_, _, e)| e.size()).sum::<usize>() + dpairs.len();
        MergeSystem {
            cl,
            pairs,
            dpairs,
            m,
            n: 3 * k_movable,
            centres: vec![[0.0; 2]; cl.len()],
            radii: vec![1.0; cl.len()],
        }
    }

    /// The same system with each moving cluster turning about its own centre, its rotation
    /// measured as arc length at `radii`.
    pub fn about(mut self, centres: Vec<[f64; 2]>, radii: Vec<f64>) -> MergeSystem<'a> {
        debug_assert_eq!(centres.len(), self.cl.len());
        debug_assert_eq!(radii.len(), self.cl.len());
        self.centres = centres;
        self.radii = radii;
        self
    }

    pub fn pose(&self, u: &[f64], ci: usize, e: El) -> Pose {
        let p = &self.cl[ci].els[&e];
        if ci == 0 {
            return p.clone();
        }
        let o = 3 * (ci - 1);
        let [cx, cy] = self.centres[ci];
        let th = u[o] / self.radii[ci];
        // rotate about the centre: R(p - c) + c + t
        apply_t(&make_t(th, u[o + 1] + cx, u[o + 2] + cy), e, &shift(e, p, -cx, -cy))
    }

    /// Jacobian of a moving cluster's pose; `None` for the reference, whose block is skipped.
    fn dpose(&self, u: &[f64], ci: usize, e: El) -> Option<Vec<[f64; 3]>> {
        if ci == 0 {
            return None;
        }
        let o = 3 * (ci - 1);
        let [cx, cy] = self.centres[ci];
        let r = self.radii[ci];
        let p = shift(e, &self.cl[ci].els[&e], -cx, -cy);
        let mut j = pose_jac(e, &p, u[o] / r, u[o + 1] + cx, u[o + 2] + cy);
        for row in j.iter_mut() {
            row[0] /= r;
        }
        Some(j)
    }

    pub fn fun(&self, u: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.m];
        let mut r = 0;
        for &(i, j, e) in self.pairs {
            let a = self.pose(u, i, e);
            let b = self.pose(u, j, e);
            for t in 0..a.len() {
                out[r] = a[t] - b[t];
                r += 1;
            }
        }
        for &(i, j, la, lb, phi) in self.dpairs {
            let na = self.pose(u, i, la);
            let nb = self.pose(u, j, lb);
            let ang = (na[0] * nb[1] - na[1] * nb[0]).atan2(na[0] * nb[0] + na[1] * nb[1]);
            out[r] = remainder(ang - phi, 2.0 * std::f64::consts::PI);
            r += 1;
        }
        out
    }

    pub fn jac(&self, u: &[f64]) -> Mat {
        let n = self.n;
        let mut j = Mat::zeros(self.m, n);
        let mut r = 0;
        for &(i, jj, e) in self.pairs {
            let ji = self.dpose(u, i, e);
            let jjm = self.dpose(u, jj, e);
            for t in 0..e.size() {
                if let Some(m) = &ji {
                    for q in 0..3 {
                        j.data[r * n + 3 * (i - 1) + q] += m[t][q];
                    }
                }
                if let Some(m) = &jjm {
                    for q in 0..3 {
                        j.data[r * n + 3 * (jj - 1) + q] -= m[t][q];
                    }
                }
                r += 1;
            }
        }
        for &(i, jj, _, _, _) in self.dpairs {
            // d angle / d theta_b = 1, / d theta_a = -1
            if jj > 0 {
                j.data[r * n + 3 * (jj - 1)] += 1.0;
            }
            if i > 0 {
                j.data[r * n + 3 * (i - 1)] -= 1.0;
            }
            r += 1;
        }
        j
    }
}

/// Plain minimum-norm Newton for the tiny merge systems (3k unknowns, warm-started at the
/// identity).  No trust region: merges are near-linear from a warm start, and the caller falls
/// back to the globalised solver if this does not converge.
fn newton_small(sys: &MergeSystem, u0: &[f64], tol: f64, max_iter: usize) -> (Vec<f64>, f64) {
    let mut u = u0.to_vec();
    let mut r = sys.fun(&u);
    for _ in 0..max_iter {
        if r.is_empty() || absmax(&r) < tol {
            break;
        }
        let neg: Vec<f64> = r.iter().map(|v| -v).collect();
        let (p, _) = min_norm_solve(&sys.jac(&u), &neg, 1e-12);
        for i in 0..u.len() {
            u[i] += p[i];
        }
        r = sys.fun(&u);
        if absmax(&p) < 1e-15 {
            break;
        }
    }
    let res = if r.is_empty() { 0.0 } else { absmax(&r) };
    (u, res)
}

/// A merge system as a `TrustRegion`, so the globalised fallback is `newton::dogleg` rather
/// than a second copy of it.  Dense and tiny: the Jacobian is rebuilt at each `jacobian_at` and
/// the products are plain matrix–vector multiplies.
struct MergeTr<'a, 'b> {
    sys: &'a MergeSystem<'b>,
    j: Mat,
}

impl TrustRegion for MergeTr<'_, '_> {
    fn n(&self) -> usize {
        self.sys.n
    }
    fn m(&self) -> usize {
        self.sys.m
    }
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.sys.fun(z));
    }
    fn jacobian_at(&mut self, z: &[f64]) {
        self.j = self.sys.jac(z);
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_t_vec(v));
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_vec(v));
    }
    fn gn_step(&mut self, r: &[f64], _g: &[f64], p: &mut [f64]) {
        let neg: Vec<f64> = r.iter().map(|v| -v).collect();
        let (step, _) = min_norm_solve(&self.j, &neg, 1e-12);
        p.copy_from_slice(&step);
    }
}

/// Globalised fallback: DogLeg on the same tiny system.
fn dogleg_small(sys: &MergeSystem, u0: &[f64], max_iter: usize) -> Vec<f64> {
    let mut u = u0.to_vec();
    let mut r = sys.fun(&u);
    if r.is_empty() || u.is_empty() {
        return u;
    }
    let mut t = MergeTr { sys, j: Mat::zeros(0, 0) };
    let tol = Tol { ftol: 1e-13, xtol: 1e-14, gtol: 1e-18 };
    newton::dogleg(&mut t, &mut u, &mut r, tol, max_iter as i32, max_iter as i32 * 4);
    u
}

/// Dimension of the rigid motions that leave every element of the cluster in place: empty 3, a
/// lone point 1, lines only and all parallel 1, otherwise 0.  (Poses are generic, so two points
/// are distinct and non-parallel lines are transversal.)
fn self_motion(c: &Cluster) -> usize {
    let mut n_pts = 0;
    let mut first_n: Option<&Pose> = None;
    for (e, pose) in &c.els {
        if e.is_point() {
            n_pts += 1;
            if n_pts >= 2 {
                return 0;
            }
        } else {
            if n_pts > 0 {
                return 0;
            }
            match first_n {
                None => first_n = Some(pose),
                Some(f) => {
                    if (f[0] * pose[1] - f[1] * pose[0]).abs() > 1e-9 {
                        return 0;
                    }
                }
            }
        }
    }
    if n_pts == 1 {
        return if first_n.is_none() { 1 } else { 0 };
    }
    if first_n.is_none() {
        3
    } else {
        1
    }
}

/* -- decomposition (topology only) -------------------------------------------------------- */

struct Decomposer {
    dirs: Dirs,
    generic: BTreeMap<El, Pose>,
    droot: BTreeMap<El, El>,
    clusters: BTreeMap<usize, Cluster>,
    of: BTreeMap<El, BTreeSet<usize>>,
    cdirs: BTreeMap<usize, BTreeMap<El, El>>,
    next_id: usize,
    rel_memo: BTreeMap<(usize, usize), (Vec<El>, Vec<(El, El, f64)>)>,
    /// Scratch for `relation_bound`, which runs tens of thousands of times per decomposition.
    el_buf: Vec<El>,
    root_buf: Vec<El>,
    rel_keys: BTreeMap<usize, BTreeSet<(usize, usize)>>,
    selfm: BTreeMap<usize, usize>,
    steps: Vec<Step>,
}

impl Decomposer {
    fn register(&mut self, cid: usize, els: &[El]) {
        for &e in els {
            self.of.entry(e).or_default().insert(cid);
            if !e.is_point() {
                let r = self.droot[&e];
                let cd = self.cdirs.entry(cid).or_default();
                cd.entry(r).or_insert(e);
            }
        }
    }

    fn add(&mut self, els: &[El], fixed: bool) -> usize {
        let cid = self.next_id;
        self.next_id += 1;
        let mut m = BTreeMap::new();
        for &e in els {
            m.insert(e, self.generic[&e].clone());
        }
        self.clusters.insert(cid, Cluster { id: cid, els: m, fixed });
        self.cdirs.insert(cid, BTreeMap::new());
        let mut sorted = els.to_vec();
        sorted.sort();
        self.register(cid, &sorted);
        cid
    }

    fn remove_cluster(&mut self, cid: usize) -> Cluster {
        let c = self.clusters.remove(&cid).unwrap();
        for e in c.els.keys() {
            if let Some(s) = self.of.get_mut(e) {
                s.remove(&cid);
            }
        }
        self.cdirs.remove(&cid);
        for key in self.rel_keys.remove(&cid).unwrap_or_default() {
            self.rel_memo.remove(&key);
        }
        c
    }

    /// What two clusters share (memoised; entries die with either cluster).
    fn pair_rel(&mut self, a: usize, b: usize) -> (Vec<El>, Vec<(El, El, f64)>) {
        let (k0, k1) = (a.min(b), a.max(b));
        let key = (k0, k1);
        if let Some(hit) = self.rel_memo.get(&key) {
            return hit.clone();
        }
        let ca = &self.clusters[&k0];
        let cb = &self.clusters[&k1];
        let (small, big) = if ca.els.len() <= cb.els.len() { (ca, cb) } else { (cb, ca) };
        let common: Vec<El> =
            small.els.keys().copied().filter(|e| big.els.contains_key(e)).collect();
        let seen: BTreeSet<El> = common
            .iter()
            .filter(|e| !e.is_point())
            .map(|e| self.droot[e])
            .collect();
        let da = self.cdirs[&k0].clone();
        let db = self.cdirs[&k1].clone();
        let (src, other) = if da.len() <= db.len() { (&da, &db) } else { (&db, &da) };
        let a_has: BTreeSet<El> = self.clusters[&k0].els.keys().copied().collect();
        let mut drels: Vec<(El, El, f64)> = Vec::new();
        for (&root, &la) in src.iter() {
            if seen.contains(&root) || !other.contains_key(&root) {
                continue;
            }
            let in_a = a_has.contains(&la);
            let la_ = if in_a { la } else { da[&root] };
            let lb_ = if in_a { db[&root] } else { la };
            let Some(phi) = self.dirs.offset(la_, lb_) else { continue };
            drels.push((la_, lb_, phi));
        }
        let res = (common, drels);
        self.rel_memo.insert(key, res.clone());
        self.rel_keys.entry(a).or_default().insert(key);
        self.rel_keys.entry(b).or_default().insert(key);
        res
    }

    /// Shared rows between clusters `ids` (positions into ids); the direction relation is oriented
    /// from `ids[i]`'s line to `ids[j]`'s line.
    fn relations(&mut self, ids: &[usize]) -> (Vec<Pair>, Vec<DPair>) {
        let mut pairs = Vec::new();
        let mut dpairs = Vec::new();
        for i in 0..ids.len() {
            for j in i + 1..ids.len() {
                let (common, drels) = self.pair_rel(ids[i], ids[j]);
                for e in common {
                    pairs.push((i, j, e));
                }
                for (la, lb, phi) in drels {
                    if self.clusters[&ids[i]].els.contains_key(&la) {
                        dpairs.push((i, j, la, lb, phi));
                    } else {
                        dpairs.push((i, j, lb, la, -phi));
                    }
                }
            }
        }
        (pairs, dpairs)
    }

    fn self_motion_of(&mut self, cid: usize) -> usize {
        if let Some(&v) = self.selfm.get(&cid) {
            return v;
        }
        let v = self_motion(&self.clusters[&cid]);
        self.selfm.insert(cid, v);
        v
    }

    fn order_ref_first(&self, ids: &[usize]) -> Vec<usize> {
        let refc = self.ref_of(ids);
        let mut out = Vec::with_capacity(ids.len());
        out.push(refc);
        out.extend(ids.iter().copied().filter(|&i| i != refc));
        out
    }

    /// An upper bound on the rank of the merge system for `ids`, without building it.
    ///
    /// Both kinds of relation are transitive, and `relations` counts them pair by pair.  An
    /// element that `m` of the clusters hold is one place, so it pins its degrees of freedom
    /// `m-1` times over — not once for each of the `m(m-1)/2` pairs that mention it.  A shared
    /// point pins two: its position.  A shared line pins its offset, one — its *direction* is
    /// counted with its direction class, because that is what the relation is: a class that `n`
    /// of the clusters carry pins `n-1` rotations whether they carry it as the same line or as
    /// parallel ones.  `Horizontal` and `Vertical` tie a line to the ground x-axis, so every
    /// levelled line in a sketch lands in one class, and `n` of them look like `n²/2` independent
    /// facts when they are `n` restatements of "this line lies along that axis".
    ///
    /// The bound must never be small: one that under-counted would report a determined merge as
    /// undetermined and silently lose it to the numeric fallback.  It is not, because the merge
    /// Jacobian's rows fall into exactly these three groups — point rows, line-offset rows,
    /// direction rows — and the rank of a union of row sets is at most the sum of their ranks.
    /// Within a group the rows of an element (or class) telescope: the pair rows for (i, k) are
    /// the sum of those for (i, j) and (j, k), so `m` holders give at most `m-1` independent ones.
    /// It is also tight where it matters: two clusters sharing one line and nothing else get 2
    /// of the 3 they need, and are never worth a factorisation — which is every pair along a
    /// levelled chain.
    fn relation_bound(&mut self, ids: &[usize]) -> usize {
        // scratch, kept between calls: this runs tens of thousands of times per decomposition
        let (mut els, mut roots) = (take(&mut self.el_buf), take(&mut self.root_buf));
        els.clear();
        roots.clear();
        for id in ids {
            els.extend(self.clusters[id].els.keys());
            roots.extend(self.cdirs[id].keys());
        }
        els.sort_unstable();
        let mut bound = repeats(&mut roots);
        for w in els.windows(2) {
            if w[0] == w[1] {
                bound += if w[0].is_point() { 2 } else { 1 };
            }
        }
        (self.el_buf, self.root_buf) = (els, roots);
        bound
    }

    /// The reference cluster of a merge: the fixed one, else the biggest.  Everything else is
    /// placed relative to it, so it is the one that keeps its pose.
    fn ref_of(&self, ids: &[usize]) -> usize {
        if let Some(f) = ids.iter().copied().find(|i| self.clusters[i].fixed) {
            return f;
        }
        let mut best = ids[0];
        for &i in ids {
            if self.clusters[&i].els.len() > self.clusters[&best].els.len() {
                best = i;
            }
        }
        best
    }

    /// Relative rigid-transform DOF left after imposing everything the clusters share
    /// (0 ⟺ the merge is determined).  Generic rank of the merge Jacobian at witness poses.
    ///
    /// The cheap bound goes first and answers almost every call; only what it cannot rule out
    /// is worth ordering the clusters and building the system for.
    fn deficiency(&mut self, ids: &[usize]) -> usize {
        let refc = self.ref_of(ids);
        let k = ids.len() - 1;
        let mut need = 3 * k;
        for &i in ids {
            if i != refc {
                need = need.saturating_sub(self.self_motion_of(i));
            }
        }
        if need == 0 {
            return 0;
        }
        let bound = self.relation_bound(ids);
        if bound < need {
            return need - bound; // cannot reach `need`; no point building the system
        }
        let ids = self.order_ref_first(ids);
        let (pairs, dpairs) = self.relations(&ids);
        let cl: Vec<Cluster> = ids.iter().map(|i| self.clusters[i].clone()).collect();
        let sys = MergeSystem::new(&cl, &pairs, &dpairs, k);
        let j = sys.jac(&vec![0.0; 3 * k]);
        let rank = if j.rows > 0 && j.cols > 0 { rank_rrqr(&j, 1e-9) } else { 0 };
        debug_assert!(rank <= bound, "relation_bound {bound} under the rank {rank}");
        need.saturating_sub(rank)
    }

    fn determined(&mut self, ids: &[usize]) -> bool {
        self.deficiency(ids) == 0
    }

    fn merge(&mut self, ids_in: &[usize]) -> usize {
        let ids = self.order_ref_first(ids_in);
        let (pairs, dpairs) = self.relations(&ids);
        let mut ppp: Option<(El, El, El)> = None;
        if ids.len() == 3
            && dpairs.is_empty()
            && pairs.len() == 3
            && pairs.iter().all(|&(_, _, e)| e.is_point())
        {
            let mut slots: BTreeSet<(usize, usize)> = BTreeSet::new();
            for &(i, j, _) in &pairs {
                slots.insert((i, j));
            }
            if slots.len() == 3 {
                let get = |i: usize, j: usize| {
                    pairs.iter().find(|&&(a, b, _)| a == i && b == j).map(|&(_, _, e)| e)
                };
                // x = ref&B, y = B&C, z = C&ref
                if let (Some(x), Some(y), Some(z)) = (get(0, 1), get(1, 2), get(0, 2)) {
                    ppp = Some((x, y, z));
                }
            }
        }
        let keep = ids[0];
        self.selfm.remove(&keep);
        for key in self.rel_keys.remove(&keep).unwrap_or_default() {
            self.rel_memo.remove(&key);
        }
        // small-into-large: absorb into the reference
        let mut kc = self.clusters.remove(&keep).unwrap();
        for &i in &ids[1..] {
            let c = self.remove_cluster(i);
            self.selfm.remove(&i);
            let mut fresh: Vec<El> = Vec::new();
            for (e, pose) in c.els {
                if let std::collections::btree_map::Entry::Vacant(slot) = kc.els.entry(e) {
                    slot.insert(pose);
                    fresh.push(e);
                }
            }
            self.clusters.insert(keep, kc);
            self.register(keep, &fresh);
            kc = self.clusters.remove(&keep).unwrap();
            kc.fixed = kc.fixed || c.fixed;
        }
        self.clusters.insert(keep, kc);
        self.steps.push(Step { ids, pairs, dpairs, ppp, branch: None });
        keep
    }

    /// The clusters that share an *element* with `a` — the only ones a merge with `a` can ever
    /// be determined with.
    ///
    /// Sharing a direction class alone does not make a neighbour.  Directions are translation
    /// invariant, so a cluster whose only tie to the others is "my line is parallel to yours"
    /// keeps both of its translations whatever else the set pins: it can never be part of a
    /// determined merge, and is never worth proposing as one.  The relation still counts once
    /// such a cluster *is* a candidate — `pair_rel` reads it off `cdirs` — it just does not
    /// nominate candidates.  Counting it as adjacency did, and was ruinous: `Horizontal` and
    /// `Vertical` put every levelled line in the ground x-axis's class, so in a levelled sketch
    /// every cluster was a neighbour of every other, disjoint components included, and the
    /// worklist tried O(N²) triples per cluster and re-queued the whole sketch after each merge.
    fn neighbours(&self, a: usize) -> BTreeSet<usize> {
        let mut nb = BTreeSet::new();
        for e in self.clusters[&a].els.keys() {
            if let Some(s) = self.of.get(e) {
                nb.extend(s.iter().copied());
            }
        }
        nb.remove(&a);
        nb
    }

    /// The clusters no other cluster strictly contains.  A container holds every element of
    /// the contained, so it is among the holders of its first one — `of` knows those, and the
    /// check is local rather than a pass over every cluster for every cluster.
    fn maximal_clusters(&self) -> Vec<usize> {
        self.clusters
            .keys()
            .copied()
            .filter(|&cid| {
                let a = &self.clusters[&cid].els;
                let Some(first) = a.keys().next() else { return true };
                !self.of[first].iter().any(|&o| {
                    if o == cid {
                        return false;
                    }
                    let b = &self.clusters[&o].els;
                    a.len() < b.len() && a.keys().all(|e| b.contains_key(e))
                })
            })
            .collect()
    }

    /// Worklist: a cluster is re-examined when it is created or a neighbour changes.
    fn tree_merges(&mut self, seed_ids: &[usize]) {
        let mut work: Vec<usize> = seed_ids.to_vec();
        let mut head = 0usize;
        let mut queued: BTreeSet<usize> = work.iter().copied().collect();
        while head < work.len() {
            let a = work[head];
            head += 1;
            queued.remove(&a);
            if !self.clusters.contains_key(&a) {
                continue;
            }
            let nbs: Vec<usize> = self.neighbours(a).into_iter().collect();
            let mut out: Option<usize> = None;
            for &b in &nbs {
                if self.determined(&[a, b]) {
                    out = Some(self.merge(&[a, b]));
                    break;
                }
            }
            if out.is_none() {
                'outer: for i in 0..nbs.len() {
                    let nb_b = self.neighbours(nbs[i]);
                    for j in i + 1..nbs.len() {
                        if nb_b.contains(&nbs[j]) && self.determined(&[a, nbs[i], nbs[j]]) {
                            out = Some(self.merge(&[a, nbs[i], nbs[j]]));
                            break 'outer;
                        }
                    }
                }
            }
            if let Some(o) = out {
                let mut refresh = vec![o];
                refresh.extend(self.neighbours(o));
                for x in refresh {
                    if queued.insert(x) {
                        work.push(x);
                    }
                }
            }
            if head > 4096 && head * 2 > work.len() {
                work.drain(0..head);
                head = 0;
            }
        }
    }

    /// Smallest rigid subset of >= 4 clusters found by greedy growth from every seed (pairs and
    /// triples are already exhausted).  `None` if nothing rigid within `core_max`.
    fn find_core(&mut self, core_max: usize) -> Option<Vec<usize>> {
        let mut best: Option<Vec<usize>> = None;
        let live = self.maximal_clusters();
        if live.len() > 400 {
            return None;
        }
        for seed in live {
            let mut s = vec![seed];
            let mut in_s: BTreeSet<usize> = BTreeSet::new();
            in_s.insert(seed);
            while s.len() < core_max
                && best.as_ref().map(|b| s.len() + 1 < b.len()).unwrap_or(true)
            {
                let mut frontier: BTreeSet<usize> = BTreeSet::new();
                for &x in &s {
                    for nb in self.neighbours(x) {
                        if !in_s.contains(&nb) {
                            frontier.insert(nb);
                        }
                    }
                }
                if frontier.is_empty() {
                    break;
                }
                let mut best_n = usize::MAX;
                let mut best_d = usize::MAX;
                let mut best_size = usize::MAX;
                for nb in frontier {
                    s.push(nb);
                    let d = self.deficiency(&s);
                    s.pop();
                    let size = self.clusters[&nb].els.len();
                    if d < best_d || (d == best_d && size < best_size) {
                        best_d = d;
                        best_size = size;
                        best_n = nb;
                    }
                }
                s.push(best_n);
                in_s.insert(best_n);
                if best_d == 0 {
                    if best.as_ref().map(|b| s.len() < b.len()).unwrap_or(true) {
                        best = Some(s.clone());
                    }
                    break;
                }
            }
        }
        best
    }
}

pub fn decompose(graph: ConstraintGraph, seed: u32, core_max: usize) -> Plan {
    let mut rng = Rng::new(seed);
    let mut dirs = Dirs::default();
    for d in &graph.dirs {
        dirs.join(d.a, d.b, d.phi);
    }
    let elements = graph.elements();
    // generic (witness) poses: random points; lines get a random normal per direction class (plus
    // their class offset) and a random offset — merge decisions are structural, so they must not
    // depend on the user's possibly-degenerate geometry
    let mut base_angle: BTreeMap<El, f64> = BTreeMap::new();
    let mut generic: BTreeMap<El, Pose> = BTreeMap::new();
    let mut droot: BTreeMap<El, El> = BTreeMap::new();
    for &e in &elements {
        if e.is_point() {
            generic.insert(e, vec![rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0)]);
        } else {
            let (root, pot) = dirs.find(e);
            droot.insert(e, root);
            let ang = *base_angle
                .entry(root)
                .or_insert_with(|| rng.uniform(0.0, 2.0 * std::f64::consts::PI))
                + pot;
            generic.insert(e, vec![ang.cos(), ang.sin(), rng.uniform(-100.0, 100.0)]);
        }
    }

    let mut d = Decomposer {
        dirs,
        generic,
        droot,
        clusters: BTreeMap::new(),
        of: elements.iter().map(|&e| (e, BTreeSet::new())).collect(),
        cdirs: BTreeMap::new(),
        next_id: 0,
        rel_memo: BTreeMap::new(),
        el_buf: Vec::new(),
        root_buf: Vec::new(),
        rel_keys: BTreeMap::new(),
        selfm: BTreeMap::new(),
        steps: Vec::new(),
    };

    let mut ground_els = vec![X_AXIS];
    ground_els.extend(graph.ground_points.iter().map(|&i| El::p(i)));
    let ground = d.add(&ground_els, true);
    let leaves: Vec<(usize, usize)> = graph
        .edges
        .iter()
        .enumerate()
        .map(|(i, e)| (d.add(&[e.a, e.b], false), i))
        .collect();
    let lone: Vec<El> =
        elements.iter().copied().filter(|e| d.of[e].is_empty()).collect();
    let singletons: Vec<(usize, El)> = lone.into_iter().map(|e| (d.add(&[e], false), e)).collect();

    let seeds: Vec<usize> = d.clusters.keys().copied().collect();
    d.tree_merges(&seeds);
    while let Some(core) = d.find_core(core_max) {
        let out = d.merge(&core);
        let mut next = vec![out];
        next.extend(d.neighbours(out));
        d.tree_merges(&next);
    }
    let roots = d.maximal_clusters();
    let mut root_els = BTreeMap::new();
    let mut root_dirs = BTreeMap::new();
    let mut roots_of: BTreeMap<El, Vec<usize>> = BTreeMap::new();
    for &r in &roots {
        let els: Vec<El> = d.clusters[&r].els.keys().copied().collect();
        for &e in &els {
            roots_of.entry(e).or_default().push(r);
        }
        root_els.insert(r, els);
        root_dirs.insert(r, d.cdirs[&r].clone());
    }
    let droot: BTreeMap<El, (El, f64)> =
        elements.iter().filter(|e| !e.is_point()).map(|&e| (e, d.dirs.find(e))).collect();
    let mut class_roots: BTreeMap<El, Vec<usize>> = BTreeMap::new();
    for (&r, dirs) in &root_dirs {
        for class in dirs.keys() {
            class_roots.entry(*class).or_default().push(r);
        }
    }
    Plan {
        graph,
        leaves,
        ground_id: ground,
        singletons,
        steps: d.steps,
        roots,
        sticky_branches: false,
        root_els,
        root_dirs,
        roots_of,
        droot,
        class_roots,
    }
}

/* -- execution ---------------------------------------------------------------------------- */

fn world_pose(g: &ConstraintGraph, sk: &Sketch, e: El) -> Pose {
    match e.kind {
        ElKind::P => {
            let (x, y) = sk.point_xy(g.class_pose_point(e.i()));
            vec![x, y]
        }
        ElKind::L => {
            if e == X_AXIS {
                x_pose()
            } else {
                line_normal(sk, g.lines[e.i()]).to_vec()
            }
        }
        ElKind::V => {
            let (ea, eb) = g.virtuals[e.i()];
            let a = world_pose(g, sk, ea);
            let b = world_pose(g, sk, eb);
            normal_of(a[0], a[1], b[0], b[1]).to_vec()
        }
    }
}

/// Poses of a 2-element cluster satisfying its edge, nearest the current geometry.
fn leaf_poses(g: &ConstraintGraph, sk: &Sketch, edge: &Edge) -> BTreeMap<El, Pose> {
    let a = world_pose(g, sk, edge.a);
    let b = world_pose(g, sk, edge.b);
    let v = g.edge_value(sk, edge);
    let mut out = BTreeMap::new();
    if edge.kind == EdgeKind::Pp {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l = dx.hypot(dy);
        let (ux, uy) = if l > 1e-12 { (dx / l, dy / l) } else { (1.0, 0.0) };
        out.insert(edge.b, vec![a[0] + v * ux, a[1] + v * uy]);
        out.insert(edge.a, a);
        return out;
    }
    // PL: n·p - c = v
    let (nx, ny, c) = (b[0], b[1], b[2]);
    let off = nx * a[0] + ny * a[1] - c - v;
    out.insert(edge.a, vec![a[0] - off * nx, a[1] - off * ny]);
    out.insert(edge.b, b);
    out
}

/// Triangle merge with all shared elements points: ref&B = {x}, B&C = {y}, C&ref = {z}.  In ref's
/// frame y is a circle-circle intersection; `sign` is the chirality — the orientation of the
/// triangle (x, z, y), invariant under rigid motions of the whole.
fn merge_ppp(
    refc: &Cluster,
    b: &Cluster,
    c: &Cluster,
    x: El,
    y: El,
    z: El,
    sign: i32,
) -> ([f64; 4], [f64; 4]) {
    let xa = &refc.els[&x];
    let za = &refc.els[&z];
    let bx = &b.els[&x];
    let by = &b.els[&y];
    let cz = &c.els[&z];
    let cy = &c.els[&y];
    let d_b = (by[0] - bx[0]).hypot(by[1] - bx[1]);
    let d_c = (cy[0] - cz[0]).hypot(cy[1] - cz[1]);
    let (ex, ey) = (za[0] - xa[0], za[1] - xa[1]);
    let l = ex.hypot(ey);
    let (ux, uy) = if l > 1e-12 { (ex / l, ey / l) } else { (1.0, 0.0) };
    let aa = if l > 1e-12 { (d_b * d_b - d_c * d_c + l * l) / (2.0 * l) } else { 0.0 };
    let h2 = d_b * d_b - aa * aa;
    let h = if h2 > 0.0 { h2.sqrt() } else { 0.0 };
    let (fx, fy) = (xa[0] + aa * ux, xa[1] + aa * uy);
    // +1: y left of x->z
    let ya = if sign > 0 {
        vec![fx - h * uy, fy + h * ux]
    } else {
        vec![fx + h * uy, fy - h * ux]
    };
    (fit2(bx, by, xa, &ya), fit2(cz, cy, za, &ya))
}

/// Write a point element's pose back to every Point of its coincidence class.
pub fn write_point(g: &ConstraintGraph, sk: &mut Sketch, e: El, pose: &[f64]) {
    if !e.is_point() {
        return;
    }
    for &p in &g.members[e.i()] {
        // a class can hold a fixed point as well as free ones; the pose came *from* the fixed
        // one, and writing it back over a fixed param would move geometry the user pinned
        let (px, py) = (sk.points[p].x as usize, sk.points[p].y as usize);
        if !sk.params[px].fixed {
            sk.params[px].value = pose[0];
        }
        if !sk.params[py].fixed {
            sk.params[py].value = pose[1];
        }
    }
}

/// Transform for an unfixed root: elements already placed by earlier roots are aligned exactly
/// (>= 2 shared points: a rigid fit on them; 1 shared point: it pins the translation and the rest
/// vote on the rotation); the remainder is least-change onto current geometry.
fn place_root(
    c: &Cluster,
    placed: &BTreeMap<El, Pose>,
    g: &ConstraintGraph,
    sk: &Sketch,
) -> [f64; 4] {
    let pts: Vec<El> = c.els.keys().copied().filter(|e| e.is_point()).collect();
    let shared: Vec<El> = pts.iter().copied().filter(|e| placed.contains_key(e)).collect();
    let xy = |p: &Pose| (p[0], p[1]);
    if shared.len() >= 2 {
        let src: Vec<(f64, f64)> = shared.iter().map(|e| xy(&c.els[e])).collect();
        let dst: Vec<(f64, f64)> = shared.iter().map(|e| xy(&placed[e])).collect();
        return procrustes(&src, &dst);
    }
    let src: Vec<(f64, f64)> = pts.iter().map(|e| xy(&c.els[e])).collect();
    let dst: Vec<(f64, f64)> = pts
        .iter()
        .map(|&e| match placed.get(&e) {
            Some(p) => xy(p),
            None => xy(&world_pose(g, sk, e)),
        })
        .collect();
    let mut t = procrustes(&src, &dst);
    if shared.len() == 1 {
        let e = shared[0];
        let moved = apply_t(&t, e, &c.els[&e]);
        t[2] += placed[&e][0] - moved[0];
        t[3] += placed[&e][1] - moved[1];
    }
    t
}

/// Replay the plan on the current sketch values and write the result back.  `capture = Some(i)`
/// returns copies of the clusters entering step i instead (no write-back).
pub fn execute(plan: &mut Plan, sk: &mut Sketch, capture: Option<usize>) -> Option<Vec<Cluster>> {
    // dimensions may have been edited since the plan was compiled; edge values are read from the
    // sketch on every execution, and the radii have to be too
    plan.graph.refresh_radii(sk);
    let mut cl: BTreeMap<usize, Cluster> = BTreeMap::new();
    {
        let g = &plan.graph;
        let mut gels: BTreeMap<El, Pose> = BTreeMap::new();
        gels.insert(X_AXIS, x_pose());
        for &i in &g.ground_points {
            gels.insert(El::p(i), world_pose(g, sk, El::p(i)));
        }
        cl.insert(plan.ground_id, Cluster { id: plan.ground_id, els: gels, fixed: true });
        for &(cid, ei) in &plan.leaves {
            cl.insert(cid, Cluster { id: cid, els: leaf_poses(g, sk, &g.edges[ei]), fixed: false });
        }
        for &(cid, e) in &plan.singletons {
            let mut m = BTreeMap::new();
            m.insert(e, world_pose(g, sk, e));
            cl.insert(cid, Cluster { id: cid, els: m, fixed: false });
        }
    }

    for si in 0..plan.steps.len() {
        let ids = plan.steps[si].ids.clone();
        let mut parts: Vec<Cluster> =
            ids.iter().map(|i| cl.remove(i).expect("plan step input")).collect();
        if capture == Some(si) {
            return Some(parts);
        }
        let ts: Vec<[f64; 4]>;
        if let Some((x, y, z)) = plan.steps[si].ppp {
            if plan.steps[si].branch.is_none() || !plan.sticky_branches {
                // sketch-guided chirality: the same signed area the drag's order-type guards watch
                let g = &plan.graph;
                let (a, b, c) =
                    (g.members[x.i()][0], g.members[z.i()][0], g.members[y.i()][0]);
                plan.steps[si].branch =
                    Some(if orientation(sk, a, b, c) >= 0.0 { 1 } else { -1 });
            }
            let sign = plan.steps[si].branch.unwrap();
            let (t1, t2) = merge_ppp(&parts[0], &parts[1], &parts[2], x, y, z, sign);
            ts = vec![t1, t2];
        } else if parts.len() > 1 {
            // identity is the natural warm start: leaves are re-derived from the current geometry,
            // so the root the sketch is on is the one nearest the identity (sticky by nature)
            let k = parts.len() - 1;
            let sys = MergeSystem::new(&parts, &plan.steps[si].pairs, &plan.steps[si].dpairs, k);
            let u0 = vec![0.0; 3 * k];
            let (mut u, res) = newton_small(&sys, &u0, 1e-13, 40);
            if res > 1e-9 {
                u = dogleg_small(&sys, &u0, 300); // cores or bad warm starts: globalise
            }
            ts = (0..k).map(|i| make_t(u[3 * i], u[3 * i + 1], u[3 * i + 2])).collect();
        } else {
            ts = Vec::new();
        }
        // absorb in place (parts were removed above)
        let movable = parts.split_off(1);
        let mut refc = parts.pop().unwrap();
        for (i, c) in movable.into_iter().enumerate() {
            for (e, pose) in &c.els {
                if !refc.els.contains_key(e) {
                    refc.els.insert(*e, apply_t(&ts[i], *e, pose));
                }
            }
            refc.fixed = refc.fixed || c.fixed;
        }
        let out = plan.steps[si].out();
        cl.insert(out, refc);
    }

    // place the roots: fixed ones are in the world frame; others least-change onto the current
    // geometry, aligning any element already placed by an earlier root exactly
    let mut placed: BTreeMap<El, Pose> = BTreeMap::new();
    let mut order = plan.roots.clone();
    order.sort_by(|&a, &b| {
        let (ca, cb) = (&cl[&a], &cl[&b]);
        (!ca.fixed as u8)
            .cmp(&(!cb.fixed as u8))
            .then(cb.els.len().cmp(&ca.els.len()))
            .then(a.cmp(&b))
    });
    for rid in order {
        let c = cl.get_mut(&rid).unwrap();
        if !c.fixed {
            let t = place_root(c, &placed, &plan.graph, sk);
            let keys: Vec<El> = c.els.keys().copied().collect();
            for e in keys {
                let np = apply_t(&t, e, &c.els[&e]);
                c.els.insert(e, np);
            }
        }
        for (e, pose) in &c.els {
            placed.entry(*e).or_insert_with(|| pose.clone());
        }
    }
    for (e, pose) in &placed {
        write_point(&plan.graph, sk, *e, pose);
    }
    let rounds: Vec<EntRef> = (0..sk.circles.len())
        .map(EntRef::circle)
        .chain((0..sk.arcs.len()).map(EntRef::arc))
        .collect();
    for e in rounds {
        let rp = sk.round_radius(e) as u32;
        if let Some(&r) = plan.graph.known_radius.get(&rp) {
            if !sk.params[rp as usize].fixed {
                sk.params[rp as usize].value = r;
            }
        }
    }
    None
}

/* -- plan solver ---------------------------------------------------------------------------- */

#[derive(Clone, Debug)]
pub struct PlanResult {
    pub success: bool,
    pub max_residual: f64,
    pub fell_back: bool,
    pub numeric: Option<SolveResult>,
    pub n_steps: usize,
}

/// The same outcome in the solver's common result type (method `plan` or the fallback's).
pub fn as_solve_result(pr: &PlanResult) -> SolveResult {
    if let Some(n) = &pr.numeric {
        return n.clone();
    }
    let mut r = SolveResult::plain("plan", pr.success, pr.max_residual, pr.n_steps as i32);
    r.message = "plan".to_string();
    r
}

/// Compile once per topology (graph + decomposition + a System for verification); `solve` replays
/// the plan and falls back to the numeric core when the residual says the plan did not (fully)
/// determine the sketch.
pub struct PlanSolver {
    pub plan: Plan,
    pub system: System,
}

impl PlanSolver {
    pub fn new(sk: &Sketch, sticky: bool) -> PlanSolver {
        let graph = build(sk);
        let mut plan = decompose(graph, 0, 12);
        plan.sticky_branches = sticky;
        let system = System::new(sk);
        PlanSolver { plan, system }
    }

    /// Flip the root of every closed-form construction that places `e`, recording the choice in the
    /// sketch (root choices are document state — `solve` reads them back).
    pub fn flip(&mut self, sk: &mut Sketch, e: El) -> usize {
        let n = self.plan.flip(e);
        if n > 0 {
            for (k, v) in self.plan.branches() {
                sk.branches.insert(k, v);
            }
        }
        n
    }

    /// Make sure `sk` satisfies its constraints, solving only if it does not already — the
    /// precondition every drag starts from, since the wave preserves consistency and cannot
    /// create it.  A document is solved after every edit, so the usual answer costs one pass
    /// over the residuals.
    pub fn ensure_solved(&mut self, sk: &mut Sketch, tol: f64, method: Method) -> bool {
        self.system.refresh_consts(sk);
        let z = self.system.z0(sk);
        self.system.max_relative_residual(&z) <= tol || self.solve(sk, tol, true, method).success
    }

    pub fn solve(&mut self, sk: &mut Sketch, tol: f64, fallback: bool, method: Method) -> PlanResult {
        // root choices are document state, read every solve — a plan cached per topology must not
        // replay a stale branch after the user flips one
        let branches = sk.branches.clone();
        self.plan.apply_branches(&branches);
        execute(&mut self.plan, sk, None);
        // dimensions may have been edited since compile
        self.system.refresh_consts(sk);
        let z = self.system.z0(sk);
        let mut mx = self.system.max_hard_residual(&z);
        let mut rel = self.system.max_relative_residual(&z);
        let mut numeric = None;
        let mut fell_back = false;
        if rel > tol && fallback {
            fell_back = true;
            // `System::solve` re-homes any curve contact and rebuilds itself if one moved to
            // another span, so this system is current whatever the fallback did
            let r = self.system.solve(sk, SolveOpts { method, ..SolveOpts::default() });
            mx = r.max_residual;
            let z = self.system.z0(sk);
            rel = self.system.max_relative_residual(&z);
            numeric = Some(r);
        }
        for (k, v) in self.plan.branches() {
            sk.branches.insert(k, v);
        }
        PlanResult {
            success: rel <= 1e-6,
            max_residual: mx,
            fell_back,
            numeric,
            n_steps: self.plan.steps.len(),
        }
    }
}

/// The closed-form merges' triangles (x, z, y) as Points — the order-type invariants to guard.
pub fn ppp_triangles(plan: &Plan) -> Vec<Triangle> {
    let g = &plan.graph;
    plan.steps
        .iter()
        .filter_map(|st| st.ppp)
        .map(|(x, y, z)| (g.members[x.i()][0], g.members[z.i()][0], g.members[y.i()][0]))
        .collect()
}

/* -- the wave: dragging as rigid motion of the plan's roots ------------------------------- */

/// Shared rows between rigid bodies: every element two of them hold, and one direction relation
/// per direction class two of them carry without holding a common line of it (a shared line
/// already says what the relation says).  `dirs[i]` is body `i`'s line per class root.
fn body_relations(
    cl: &[Cluster],
    dirs: &[BTreeMap<El, El>],
    droot: &BTreeMap<El, (El, f64)>,
) -> (Vec<Pair>, Vec<DPair>) {
    let mut pairs = Vec::new();
    let mut dpairs = Vec::new();
    for i in 0..cl.len() {
        for j in i + 1..cl.len() {
            let (small, big) =
                if cl[i].els.len() <= cl[j].els.len() { (&cl[i], &cl[j]) } else { (&cl[j], &cl[i]) };
            let mut seen: BTreeSet<El> = BTreeSet::new();
            for e in small.els.keys() {
                if big.els.contains_key(e) {
                    pairs.push((i, j, *e));
                    if !e.is_point() {
                        seen.insert(droot[e].0);
                    }
                }
            }
            for (class, &la) in &dirs[i] {
                if seen.contains(class) {
                    continue;
                }
                if let Some(&lb) = dirs[j].get(class) {
                    dpairs.push((i, j, la, lb, droot[&lb].1 - droot[&la].1));
                }
            }
        }
    }
    (pairs, dpairs)
}

/// How far turning is from sliding: a body's rotation is measured as arc length at this many
/// radii of gyration, so a free body pulled at one point rides along with the cursor rather
/// than spinning about its centre — a turn that displaces the body as much as a slide would
/// costs this much more in the norm — while a body whose anchors leave it no other way still
/// turns, and exactly.
const TURN_COST: f64 = 16.0;

/// Where a body turns and what its turning is measured at: the mean of its points and
/// `TURN_COST` times their radius of gyration; a body without points turns about the foot of
/// its first line, at unit scale.
fn centre_and_radius(c: &Cluster) -> ([f64; 2], f64) {
    let (mut x, mut y, mut n) = (0.0, 0.0, 0usize);
    for (e, p) in &c.els {
        if e.is_point() {
            x += p[0];
            y += p[1];
            n += 1;
        }
    }
    if n == 0 {
        return match c.els.iter().find(|(e, _)| !e.is_point()) {
            Some((_, p)) => ([p[0] * p[2], p[1] * p[2]], 1.0),
            None => ([0.0, 0.0], 1.0),
        };
    }
    let (cx, cy) = (x / n as f64, y / n as f64);
    let mut r2 = 0.0;
    for (e, p) in &c.els {
        if e.is_point() {
            r2 += (p[0] - cx).powi(2) + (p[1] - cy).powi(2);
        }
    }
    let r = (r2 / n as f64).sqrt();
    ([cx, cy], if r > 0.0 { TURN_COST * r } else { 1.0 })
}

/// How many roots a frame may move as rigid bodies before the dense merge system stops being
/// tiny and the numeric drag on the part is the better tool.
const WAVE_MAX: usize = 48;

/// What one wave step did.
struct WaveOutcome {
    /// The dragged point landed on the cursor.
    reached: bool,
    /// A rigid-body solve failed to make the region consistent (nonlinearity), or the region
    /// outgrew `WAVE_MAX`: the numeric drag has to take over.
    failed: bool,
}

/// A drag as the plan sees it: the roots are rigid bodies, and moving one is a rigid motion.
///
/// The roots holding the dragged point form the *region*.  Every element a region root shares
/// with a root outside it is an anchor, held where it is; every direction class a region root
/// carries that some outside root carries too is an anchor on its rotation.  The region's bodies
/// are moved by the tiny merge system over them — pull (the cursor is a row) then polish (anchors
/// only, minimum-norm from the pulled pose, so it is least-change and exact) — and if the cursor
/// is not reached and a neighbouring root is free to help, the region grows by it and the solve
/// is repeated.  That is where the locality is: a frame costs the region, which on a chain of
/// levelled segments is three corners however long the chain, and a body that nothing pulls on
/// is never looked at.  The region is kept across frames; within a gesture it only grows.
struct Wave {
    el: El,
    region: BTreeSet<usize>,
    /// The region's bodies, posed.  Read from the sketch when a root joins the region and
    /// advanced by each frame's transforms after that — never re-read, so a body stays the
    /// exact rigid copy of what the solve made it, and nothing compounds.
    bodies: BTreeMap<usize, Cluster>,
    /// What the region shares with the rest, at the pose it had when it became shared — held
    /// there, and never re-read either: a line through a region point would otherwise tilt by
    /// the tolerance every frame and the region would follow its own error.
    anchors: BTreeMap<El, Pose>,
    /// How close to the cursor is "on it".
    reach: f64,
    /// How small a body-system residual is "consistent" — in the part's own units, since the
    /// rows mix unit normals with offsets and positions that are lengths.
    tol: f64,
}

impl Wave {
    fn new(plan: &Plan, el: El, extent: f64) -> Wave {
        let extent = extent.max(1.0);
        let mut w = Wave {
            el,
            region: BTreeSet::new(),
            bodies: BTreeMap::new(),
            anchors: BTreeMap::new(),
            reach: 1e-7 * extent,
            tol: 1e-9 * extent,
        };
        w.region = w.seed(plan);
        w
    }

    /// The roots the dragged point is in, less the ground (which does not move).
    fn seed(&self, plan: &Plan) -> BTreeSet<usize> {
        plan.roots_of
            .get(&self.el)
            .map(|v| v.iter().copied().filter(|&r| r != plan.ground_id).collect())
            .unwrap_or_default()
    }

    /// The free roots next to the region: those holding an element of it.  A root tied to the
    /// region by a direction class alone is not one — it stays where it is and pins the
    /// rotation of what it is tied to, the way the ground's x-axis pins every levelled body.
    fn frontier(&self, plan: &Plan) -> BTreeSet<usize> {
        let mut out = BTreeSet::new();
        for &r in &self.region {
            for e in &plan.root_els[&r] {
                out.extend(plan.roots_of[e].iter().copied());
            }
        }
        out.remove(&plan.ground_id);
        for r in &self.region {
            out.remove(r);
        }
        out
    }

    /// Move the region toward the cursor, growing it as needed, and write the result.
    fn step(&mut self, plan: &Plan, sk: &mut Sketch, x: f64, y: f64) -> WaveOutcome {
        let g = &plan.graph;
        if self.region.is_empty() {
            return WaveOutcome { reached: false, failed: false };
        }
        loop {
            if self.region.len() > WAVE_MAX {
                return WaveOutcome { reached: false, failed: true };
            }
            let ids: Vec<usize> = self.region.iter().copied().collect();
            // the bodies: a root joining the region is read from the sketch, where nothing has
            // moved it yet — except what it shared with the region, which is held at the anchor
            let mut cl: Vec<Cluster> = Vec::with_capacity(ids.len() + 1);
            let mut dirs: Vec<BTreeMap<El, El>> = Vec::with_capacity(ids.len() + 1);
            cl.push(Cluster { id: usize::MAX, els: BTreeMap::new(), fixed: true });
            dirs.push(BTreeMap::new());
            for &r in &ids {
                let body = self.bodies.entry(r).or_insert_with(|| {
                    let els = plan.root_els[&r].iter().map(|&e| {
                        (e, self.anchors.get(&e).cloned().unwrap_or_else(|| world_pose(g, sk, e)))
                    });
                    Cluster { id: r, els: els.collect(), fixed: false }
                });
                cl.push(body.clone());
                dirs.push(plan.root_dirs[&r].clone());
            }
            // the anchors: what the region shares with the rest, held where it is
            let mut shared: BTreeSet<El> = BTreeSet::new();
            for &r in &ids {
                for &e in &plan.root_els[&r] {
                    if plan.roots_of[&e].iter().any(|o| !self.region.contains(o)) {
                        shared.insert(e);
                    }
                }
            }
            self.anchors.retain(|e, _| shared.contains(e));
            for &e in &shared {
                let pose = self.anchors.entry(e).or_insert_with(|| world_pose(g, sk, e)).clone();
                cl[0].els.insert(e, pose);
                if !e.is_point() {
                    dirs[0].entry(plan.droot[&e].0).or_insert(e);
                }
            }
            for &r in &ids {
                for class in plan.root_dirs[&r].keys() {
                    if dirs[0].contains_key(class) {
                        continue;
                    }
                    // a direction the rest of the sketch carries: one outside line stands for it
                    let outside = plan.class_roots[class]
                        .iter()
                        .find(|o| !self.region.contains(o))
                        .map(|o| plan.root_dirs[o][class]);
                    if let Some(l) = outside {
                        cl[0].els.entry(l).or_insert_with(|| world_pose(g, sk, l));
                        dirs[0].insert(*class, l);
                    }
                }
            }
            let k = ids.len();
            let (pairs, dpairs) = body_relations(&cl, &dirs, &plan.droot);
            // each body turns about its own centroid, so least-norm is least motion of it
            let (centres, radii): (Vec<[f64; 2]>, Vec<f64>) =
                cl.iter().map(centre_and_radius).unzip();
            let u0 = vec![0.0; 3 * k];
            // pull: the cursor is one more anchor row, on every body holding the point.  It goes
            // into the anchors in place and comes out again — a copy of the region's poses to
            // hold one extra entry is the largest allocation a frame could make.
            let mut ppairs = pairs.clone();
            for (i, &r) in ids.iter().enumerate() {
                if plan.root_els[&r].contains(&self.el) {
                    ppairs.push((0, i + 1, self.el));
                }
            }
            let held = cl[0].els.insert(self.el, vec![x, y]);
            let u = {
                let sys = MergeSystem::new(&cl, &ppairs, &dpairs, k)
                    .about(centres.clone(), radii.clone());
                let (u, res) = newton_small(&sys, &u0, self.tol * 1e-4, 40);
                if res > self.tol { dogleg_small(&sys, &u0, 200) } else { u }
            };
            match held {
                Some(p) => cl[0].els.insert(self.el, p),
                None => cl[0].els.remove(&self.el),
            };
            // polish: the anchors exactly, least change from the pulled pose
            let sys = MergeSystem::new(&cl, &pairs, &dpairs, k).about(centres, radii);
            let (u, res) = newton_small(&sys, &u, self.tol * 1e-4, 40);
            let u = if res > self.tol {
                let u2 = dogleg_small(&sys, &u, 200);
                if absmax(&sys.fun(&u2)) > self.tol {
                    return WaveOutcome { reached: false, failed: true };
                }
                u2
            } else {
                u
            };
            let reached = ids.iter().enumerate().all(|(i, &r)| {
                !plan.root_els[&r].contains(&self.el) || {
                    let p = sys.pose(&u, i + 1, self.el);
                    (p[0] - x).hypot(p[1] - y) <= self.reach
                }
            });
            if !reached {
                let more = self.frontier(plan);
                if !more.is_empty() {
                    self.region.extend(more);
                    continue;
                }
            }
            // advance the bodies and write what moved; an anchor is held, not written
            for (i, &r) in ids.iter().enumerate() {
                let body = self.bodies.get_mut(&r).unwrap();
                for (e, pose) in body.els.iter_mut() {
                    *pose = sys.pose(&u, i + 1, *e);
                    if e.is_point() && !cl[0].els.contains_key(e) {
                        write_point(g, sk, *e, pose);
                    }
                }
            }
            return WaveOutcome { reached, failed: false };
        }
    }
}

/// The dragged point's part of the document, and the point as the part numbers it — what both a
/// drag of its own and a hand-over to the numeric drag start from.
fn part_around(doc: &Sketch, point: usize) -> (Part, usize) {
    let part = Part::around(doc, EntRef::point(point));
    let p = part.point_in(point).expect("the dragged point is in its own part");
    (part, p)
}

/// DCM-style drag: the plan's roots move as rigid bodies (`Wave`), so a frame costs the region
/// of the sketch the drag reaches and nothing else — no graph analysis while dragging, recorded
/// roots stay as they are, and under-constrained bodies move least.  Large cursor jumps are taken
/// in increments so the solution tracks its branch.  Where the plan cannot carry the drag
/// (unsupported constraints, a region past `WAVE_MAX`, a body solve that does not converge) the
/// numeric pull/polish `Drag` takes over.
///
/// Made `on` the document's own plan — cached per topology by whoever owns the document — a drag
/// starts in the time it takes to find the roots holding the point, and runs on the document
/// directly.  Made with `new`, it builds its own: the dragged point's *part* of the document
/// (`io::Part`), which is all a drag can move, and a plan over that, writing each frame back —
/// so the document is never recompiled or restructured by a drag either way, and its unrelated
/// figures cost the drag nothing.
///
/// Either way the numeric fallback works on a part, so `part` is what the drag writes back and
/// what maps its point indices to the document's: `None` while the wave has the document itself.
pub struct PlanDrag {
    /// What the drag moves, when that is not the document: its own part, or the one taken for
    /// the numeric fallback.  Point indices below are this sketch's.
    part: Option<Part>,
    /// The drag's own plan over its own part (`new`); `None` for one made `on` a plan.
    solver: Option<PlanSolver>,
    /// The shape of the plan this drag was made on — the one it must be handed back.
    shape: (usize, usize, usize),
    numeric: Option<Drag>,
    /// The dragged point, as the sketch the wave runs on numbers it.
    pub point: usize,
    wave: Wave,
    max_step: f64,
    guards: Option<Vec<Triangle>>,
}

impl PlanDrag {
    /// A drag with a plan of its own, over the dragged point's part of the document.
    pub fn new(
        doc: &Sketch,
        point: usize,
        x: f64,
        y: f64,
        guards: Option<Vec<Triangle>>,
        max_step_rel: f64,
    ) -> PlanDrag {
        // continuation increments are cursor motion relative to the drawing, so the document's
        // extent sets them, whatever the part's own size
        let max_step = max_step_rel * doc.extent().max(1.0);
        let (mut part, point) = part_around(doc, point);
        // `None` still means "not computed": the numeric fallback derives them if it is reached
        let guards = guards.map(|g| part.triangles_in(&g));
        let sk = &mut part.sketch;
        let mut solver = PlanSolver::new(sk, true);
        let usable = solver.ensure_solved(sk, 1e-9, Method::DogLeg)
            && solver.plan.graph.unsupported.is_empty();
        let wave = Wave::new(&solver.plan, solver.plan.graph.point_el(point), sk.extent());
        let shape = solver.plan.shape();
        let mut d = PlanDrag {
            part: Some(part),
            solver: Some(solver),
            shape,
            numeric: None,
            point,
            wave,
            max_step,
            guards,
        };
        if !usable {
            d.hand_over(doc, None, x, y);
        }
        d
    }

    /// A drag on the document's own plan, which must be the plan of the document as it is — the
    /// same plan has to come back with every `move_to`, `guard_triangles` and `branches`.
    pub fn on(
        doc: &mut Sketch,
        plan: &mut PlanSolver,
        point: usize,
        x: f64,
        y: f64,
        guards: Option<Vec<Triangle>>,
        max_step_rel: f64,
    ) -> PlanDrag {
        let max_step = max_step_rel * doc.extent().max(1.0);
        let usable = plan.ensure_solved(doc, 1e-9, Method::DogLeg)
            && plan.plan.graph.unsupported.is_empty();
        let wave = Wave::new(&plan.plan, plan.plan.graph.point_el(point), doc.extent());
        let mut d = PlanDrag {
            part: None,
            solver: None,
            shape: plan.plan.shape(),
            numeric: None,
            point,
            wave,
            max_step,
            guards,
        };
        if !usable {
            d.hand_over(doc, Some(&plan.plan), x, y);
        }
        d
    }

    /// The plan the wave runs on: the drag's own, else the one it was made on.
    fn plan<'a>(&'a self, given: Option<&'a Plan>) -> &'a Plan {
        match &self.solver {
            Some(s) => &s.plan,
            None => {
                let p = given.expect("a drag made on the document's plan needs it back");
                debug_assert_eq!(p.shape(), self.shape, "a different plan came back to the drag");
                p
            }
        }
    }

    /// The drag's own plan, when it has one (`new`, not `on`) — the tests' way in.
    pub fn own_plan(&self) -> Option<&PlanSolver> {
        self.solver.as_ref()
    }

    /// The part the drag moves, when it is not moving the document itself.
    pub fn part(&self) -> Option<&Part> {
        self.part.as_ref()
    }

    /// Copy what moved into the document — a no-op when the drag moves it directly.
    fn write_back(&self, doc: &mut Sketch) {
        match &self.part {
            Some(p) => p.write_back(doc),
            // the wave moved the document itself, parameter by parameter, so nothing has been
            // through `Sketch::set_x` — see `Part::write_back` for the same reason
            None => crate::expr::sync_free(doc),
        }
    }

    /// A point of the stage, as the document numbers it.
    fn point_out(&self, p: usize) -> usize {
        self.part.as_ref().map(|part| part.point_out(p)).unwrap_or(p)
    }

    /// True while the plan is driving the drag (false once it handed over).
    pub fn usable(&self) -> bool {
        self.numeric.is_none()
    }

    /// Order-type invariants for the numeric path: the closed-form triangles of the plan, as the
    /// stage numbers them.
    pub fn guard_triangles(&mut self, plan: Option<&Plan>) -> Vec<Triangle> {
        if self.guards.is_none() {
            self.guards = Some(ppp_triangles(self.plan(plan)));
        }
        self.guards.clone().unwrap()
    }

    /// Hand over to the numeric pull/polish drag, from where the geometry is now.  A drag that
    /// was moving the document takes a part of it first: the numeric drag compiles systems, and
    /// those are what must not be the document's size.
    fn hand_over(&mut self, doc: &Sketch, plan: Option<&Plan>, x: f64, y: f64) {
        let guards = self.guard_triangles(plan);
        if self.part.is_none() {
            let (part, point) = part_around(doc, self.point);
            self.guards = Some(part.triangles_in(&guards));
            self.part = Some(part);
            self.point = point;
        }
        let guards = self.guards.clone().expect("guard_triangles filled them in");
        let sketch = &mut self.part.as_mut().expect("a numeric drag runs on a part").sketch;
        let mut drag = Drag::new(sketch, self.point, x, y, Method::DogLeg, 1.0, guards, 0.05);
        drag.max_step = self.max_step;
        self.numeric = Some(drag);
    }

    /// One frame.  `plan` is the one the drag was made `on`, or `None` for a drag of its own.
    pub fn move_to(
        &mut self,
        doc: &mut Sketch,
        plan: Option<&Plan>,
        x: f64,
        y: f64,
    ) -> SolveResult {
        if self.numeric.is_some() {
            return self.numeric_move(doc, x, y);
        }
        // the plan, the stage and the wave are three disjoint borrows, so they are taken from
        // the fields directly — `self.plan(given)` would borrow the whole drag
        let pl: &Plan = match &self.solver {
            Some(s) => &s.plan,
            None => {
                let p = plan.expect("a drag made on the document's plan needs it back");
                debug_assert_eq!(p.shape(), self.shape, "a different plan came back to the drag");
                p
            }
        };
        let wave = &mut self.wave;
        let sk = match &mut self.part {
            Some(p) => &mut p.sketch,
            None => doc,
        };
        let (px, py) = sk.point_xy(self.point);
        let path = increments(px, py, x, y, self.max_step);
        let mut reached = true;
        let mut failed = false;
        for &(tx, ty) in &path {
            // a failed step writes nothing, so there is nothing to roll back
            let out = wave.step(pl, sk, tx, ty);
            reached = out.reached;
            if out.failed {
                failed = true;
                break;
            }
        }
        self.write_back(doc);
        if failed {
            self.hand_over(doc, plan, px, py);
            return self.numeric_move(doc, x, y);
        }
        let mut r = SolveResult::plain("plan", true, 0.0, path.len() as i32);
        r.message =
            if reached { "plan-drag" } else { "plan-drag: held by constraints" }.to_string();
        r
    }

    fn numeric_move(&mut self, doc: &mut Sketch, x: f64, y: f64) -> SolveResult {
        let part = self.part.as_mut().expect("a numeric drag runs on a part");
        let drag = self.numeric.as_mut().expect("the numeric drag");
        let r = drag.move_to(&mut part.sketch, x, y);
        part.write_back(doc);
        r
    }

    pub fn end(&mut self) {
        if let (Some(drag), Some(part)) = (&mut self.numeric, &mut self.part) {
            drag.end(&mut part.sketch);
        }
    }

    /// Triangles whose orientation flipped during the drag, as the document numbers them.
    pub fn flips(&self) -> Vec<Triangle> {
        let Some(n) = &self.numeric else { return Vec::new() };
        n.flips
            .iter()
            .map(|&(a, b, c)| (self.point_out(a), self.point_out(b), self.point_out(c)))
            .collect()
    }

    /// The plan's recorded root choices, keyed as the document keys them.
    pub fn branches(&self, plan: Option<&Plan>) -> BTreeMap<String, i32> {
        let b = self.plan(plan).branches();
        match &self.part {
            Some(p) => p.branches_out(&b),
            None => b,
        }
    }
}
