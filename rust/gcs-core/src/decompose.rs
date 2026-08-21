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
use crate::graph::dulmage_mendelsohn;
use crate::linalg::{absmax, min_norm_solve, rank_rrqr, Mat};
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
}

impl Plan {
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

/// Residual/Jacobian for the transforms of `cl[1..]` (`cl[0]` is the reference, identity).
pub struct MergeSystem<'a> {
    pub cl: &'a [Cluster],
    pub pairs: &'a [Pair],
    pub dpairs: &'a [DPair],
    pub m: usize,
    pub n: usize,
}

impl<'a> MergeSystem<'a> {
    pub fn new(
        cl: &'a [Cluster],
        pairs: &'a [Pair],
        dpairs: &'a [DPair],
        k_movable: usize,
    ) -> MergeSystem<'a> {
        let m = pairs.iter().map(|&(_, _, e)| e.size()).sum::<usize>() + dpairs.len();
        MergeSystem { cl, pairs, dpairs, m, n: 3 * k_movable }
    }

    fn pose(&self, u: &[f64], ci: usize, e: El) -> Pose {
        let p = &self.cl[ci].els[&e];
        if ci == 0 {
            p.clone()
        } else {
            let o = 3 * (ci - 1);
            apply_t(&make_t(u[o], u[o + 1], u[o + 2]), e, p)
        }
    }

    /// Jacobian of a moving cluster's pose; `None` for the reference, whose block is skipped.
    fn dpose(&self, u: &[f64], ci: usize, e: El) -> Option<Vec<[f64; 3]>> {
        if ci == 0 {
            return None;
        }
        let o = 3 * (ci - 1);
        Some(pose_jac(e, &self.cl[ci].els[&e], u[o], u[o + 1], u[o + 2]))
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
    dir_of: BTreeMap<El, BTreeSet<usize>>,
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
                self.dir_of.entry(r).or_default().insert(cid);
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
        if let Some(cd) = self.cdirs.remove(&cid) {
            for r in cd.keys() {
                if let Some(s) = self.dir_of.get_mut(r) {
                    s.remove(&cid);
                }
            }
        }
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
    /// element that `m` of the clusters hold is one place, so it pins `2(m-1)` degrees of
    /// freedom — not two for each of the `m(m-1)/2` pairs that mention it.  A direction class
    /// that `n` of them carry pins `n-1` rotations, for the same reason: `Horizontal` and
    /// `Vertical` tie a line to the ground x-axis, so every levelled line in a sketch lands in
    /// one class, and `n` of them look like `n²/2` independent facts when they are `n`
    /// restatements of "this line lies along that axis".
    ///
    /// Not the tightest bound available: where two clusters share a *line*, its direction is
    /// counted here as well as its position, which `pair_rel` knows to drop.  That makes the
    /// bound generous, never small — a bound that under-counted would report a determined merge
    /// as undetermined and silently lose it to the numeric fallback.
    fn relation_bound(&mut self, ids: &[usize]) -> usize {
        // scratch, kept between calls: this runs tens of thousands of times per decomposition
        let (mut els, mut roots) = (take(&mut self.el_buf), take(&mut self.root_buf));
        els.clear();
        roots.clear();
        for id in ids {
            els.extend(self.clusters[id].els.keys());
            roots.extend(self.cdirs[id].keys());
        }
        let bound = 2 * repeats(&mut els) + repeats(&mut roots);
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

    fn neighbours(&self, a: usize) -> BTreeSet<usize> {
        let mut nb = BTreeSet::new();
        for e in self.clusters[&a].els.keys() {
            if let Some(s) = self.of.get(e) {
                nb.extend(s.iter().copied());
            }
        }
        if let Some(cd) = self.cdirs.get(&a) {
            for r in cd.keys() {
                if let Some(s) = self.dir_of.get(r) {
                    nb.extend(s.iter().copied());
                }
            }
        }
        nb.remove(&a);
        nb
    }

    fn maximal_clusters(&self) -> Vec<usize> {
        let ids: Vec<usize> = self.clusters.keys().copied().collect();
        ids.iter()
            .copied()
            .filter(|&cid| {
                let a = &self.clusters[&cid].els;
                !ids.iter().any(|&o| {
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
        dir_of: BTreeMap::new(),
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
    Plan { graph, leaves, ground_id: ground, singletons, steps: d.steps, roots, sticky_branches: false }
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

/// DCM-style drag: the dragged point joins the ground (fixed at the cursor) and the cached plan
/// replays per frame — no graph analysis while dragging, recorded roots are sticky, and
/// under-constrained roots move least.  Large cursor jumps are taken in increments so the solution
/// tracks its branch.  If the plan cannot determine the sketch with the point pinned (fully
/// constrained sketches, unsupported constraints) the numeric pull/polish `Drag` takes over.
pub struct PlanDrag {
    pub solver: PlanSolver,
    pub numeric: Option<Drag>,
    pub point: usize,
    max_step: f64,
    guards: Option<Vec<Triangle>>,
}

impl PlanDrag {
    pub fn new(
        sk: &mut Sketch,
        point: usize,
        x: f64,
        y: f64,
        guards: Option<Vec<Triangle>>,
        max_step_rel: f64,
    ) -> PlanDrag {
        let max_step = max_step_rel * sk.extent().max(1.0);
        let x0 = sk.get_x();
        let was = sk.point_fixed(point);
        sk.fix_point(point, true);
        let mut solver = PlanSolver::new(sk, true);
        // the plan can drive the drag iff it understands every constraint, pinning the point does
        // not over-determine the sketch, and the replay reproduces the configuration
        let over = {
            let (adj, _) = solver.system.structure();
            adj.len() > dulmage_mendelsohn(&adj, solver.system.n_free).rank
        };
        let (px, py) = sk.point_xy(point);
        let usable = solver.plan.graph.unsupported.is_empty()
            && !over
            && replay(&mut solver, sk, point, px, py) <= 1e-9;
        sk.fix_point(point, was);
        let mut d = PlanDrag { solver, numeric: None, point, max_step, guards };
        if !usable {
            sk.set_x(&x0);
            let g = d.guard_triangles(sk);
            d.numeric =
                Some(Drag::new(sk, point, x, y, Method::DogLeg, 1.0, g, 0.05));
        }
        d
    }

    /// True while the cached plan is driving the drag (false once it handed over).
    pub fn usable(&self) -> bool {
        self.numeric.is_none()
    }

    /// Order-type invariants for the numeric path: the closed-form triangles of the sketch's own
    /// (unpinned) plan.  Computed at most once per drag — never inside a move.
    pub fn guard_triangles(&mut self, sk: &Sketch) -> Vec<Triangle> {
        if self.guards.is_none() {
            self.guards = Some(ppp_triangles(&decompose(build(sk), 0, 12)));
        }
        self.guards.clone().unwrap()
    }

    pub fn move_to(&mut self, sk: &mut Sketch, x: f64, y: f64) -> SolveResult {
        if let Some(n) = &mut self.numeric {
            return n.move_to(sk, x, y);
        }
        let x_prev = sk.get_x();
        let (px, py) = sk.point_xy(self.point);
        let path = increments(px, py, x, y, self.max_step);
        let mut mx = 0.0;
        for &(tx, ty) in &path {
            mx = replay(&mut self.solver, sk, self.point, tx, ty);
            if mx > 1e-6 {
                // the plan cannot follow (a limit of the geometry was hit): hand over to the
                // numeric drag from the last good state
                sk.set_x(&x_prev);
                let g = self.guard_triangles(sk);
                let mut d = Drag::new(sk, self.point, px, py, Method::DogLeg, 1.0, g, 0.05);
                let r = d.move_to(sk, x, y);
                self.numeric = Some(d);
                return r;
            }
        }
        let mut r = SolveResult::plain("plan", true, mx, path.len() as i32);
        r.message = "plan-drag".to_string();
        r
    }

    pub fn end(&mut self, sk: &mut Sketch) {
        if let Some(n) = &mut self.numeric {
            n.end(sk);
        }
    }

    pub fn flips(&self) -> Vec<Triangle> {
        self.numeric.as_ref().map(|n| n.flips.clone()).unwrap_or_default()
    }

    pub fn branches(&self) -> BTreeMap<String, i32> {
        self.solver.plan.branches()
    }
}

/// Replay the plan with `point` pinned at (x, y); the worst hard residual, relative to its own
/// row's units, so one threshold judges every kernel.
fn replay(solver: &mut PlanSolver, sk: &mut Sketch, point: usize, x: f64, y: f64) -> f64 {
    let (px, py) = (sk.points[point].x as usize, sk.points[point].y as usize);
    sk.params[px].value = x;
    sk.params[py].value = y;
    execute(&mut solver.plan, sk, None);
    let z = solver.system.z0(sk);
    solver.system.max_relative_residual(&z)
}
