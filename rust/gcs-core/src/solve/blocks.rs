//! The block-triangular solve (docs/block-triangular-solve-plan.md): each strongly
//! connected block of the equations minimised in order, its root searched for where it
//! comes to none or to one a bound it decides refuses (`Search`), then a whole-system
//! polish; and the rescue of a whole-system DogLeg that did not settle by it.

use crate::linalg::Mat;
use crate::model::Sketch;
use crate::newton::{self, Info, Method, TrustRegion};
use crate::space;
use crate::sparse::Ata;
use crate::system::{SolveBlock, Subset, System, DENSE_MAX};

use super::bounds::{across, bound_reading, bound_tol, movable};
use super::{reads_pose, SolveOpts, SolveResult};

/// The step, relative to the block's own unknowns, below which a block's minimisation stops
/// (`newton::Tol::xtol`).  A block is accepted only at the caller's `acceptance_tol`, and its few
/// columns make the whole solve's 1e-12 a coarse limit: an angle on a short line at 180° needs a
/// step of a few 1e-12 mm to take its last 4e-12 radians, which that stops short of — as the
/// whole-system polish does from there, with a norm the whole vector's size behind its `xtol`.
const BLOCK_XTOL: f64 = 1e-15;

impl System {
    /// The block-triangular rescue of a whole-system DogLeg that did not settle: the block pass
    /// and its polish, tried from one start after another, each kept only if it clears its own
    /// bar.  A system whose seeds are good only upstream of a block can fail whole and solve in
    /// order, each block from what the blocks before it made; and a DogLeg accepted on its
    /// iteration limit, still descending in an ill-conditioned direction, can be a thousandth of
    /// the extent from the solution (docs/iteration-limit-rescue-plan.md).  So:
    ///
    /// * **a failure** is tried from the start, kept if it succeeds;
    /// * **a stop on the limit** is tried from the stop, kept if it converges there (status 0) —
    ///   the root the DogLeg was making for, finished — and a pass that stalls again (a step or
    ///   gradient stop above the tolerance) says the stop is in a basin with no solution in it,
    ///   so it is tried from the start, kept if it settles.
    ///
    /// Nothing clearing its bar, the first pass that settled is kept; nothing settling, the
    /// DogLeg's own pose, status and success stand bit for bit — a failing document's pose is
    /// what the diagnosis reads.  Whether `self`'s x is still `z`'s is the answer: true exactly
    /// when the pass kept is the last one run.
    pub(super) fn block_rescue(
        &mut self,
        sk: &Sketch,
        z0: &[f64],
        z: &mut Vec<f64>,
        res: &mut SolveResult,
        rel: &mut f64,
        opts: &SolveOpts,
    ) -> bool {
        type Bar = fn(&SolveResult) -> bool;
        let stop = z.clone();
        let tries: Vec<(&[f64], Bar)> = if res.success {
            vec![(&stop[..], |r| r.settled() && r.status == 0), (z0, SolveResult::settled)]
        } else {
            vec![(z0, |r| r.success)]
        };
        let mut passes = Vec::new();
        for (from, bar) in tries {
            let mut zb = from.to_vec();
            let (rb, relb) = self.blocks_then_polish(sk, &mut zb, opts);
            if bar(&rb) {
                (*z, *res, *rel) = (zb, rb, relb);
                return true;
            }
            passes.push((zb, rb, relb));
        }
        if let Some(pass) = passes.into_iter().find(|(_, rb, _)| rb.settled()) {
            (*z, *res, *rel) = pass;
        }
        false
    }

    /// The block pass from `z`, then the whole-system DogLeg from where it ends — the polish,
    /// which is the result.  A block is accepted when its rows meet `acceptance_tol` and every
    /// bound it decides holds (`Search`); where it does not, its root is searched for, and the
    /// first that does not come out so ends the pass, and the polish starts from the blocks solved
    /// so far (and whatever that one's own minimisation made of it).  With fewer than two blocks
    /// there is no order to exploit and this is the whole-system DogLeg alone.
    pub(super) fn blocks_then_polish(
        &mut self,
        sk: &Sketch,
        z: &mut [f64],
        opts: &SolveOpts,
    ) -> (SolveResult, f64) {
        let order = self.block_order();
        let (mut nfev, mut njev) = (0, 0);
        if order.blocks.len() >= 2 {
            // a constant read off the pose — the directions across a line its rows are taken
            // along — is read where the blocks before put it, not where it was seeded: a line
            // turned far from its seed admits a root no word says (#217); the compiled ones are
            // put back for the polish, which starts on the pass's solution, so a pass not kept
            // leaves the system as it found it
            let framed = reads_pose(sk);
            let bounded = sk.constraints.iter().any(|c| c.bound.is_some());
            let compiled: Vec<Vec<f64>> = if framed {
                self.blocks.iter().map(|b| b.consts.clone()).collect()
            } else {
                Vec::new()
            };
            // the scratch sketch the search reads, made the first time a block needs it
            let mut search: Option<Search> = None;
            let mut solved = std::collections::BTreeSet::new();
            for b in &order.blocks {
                if framed {
                    let s = search.get_or_insert_with(|| Search::new(self, sk));
                    s.sk.set_x(&self.full_x(z));
                    self.refresh_consts(&s.sk);
                }
                let sub = self.subset(&b.instances, &b.cols);
                let start: Vec<f64> = b.cols.iter().map(|&c| z[c]).collect();
                let (mut ok, info) = self.solve_block(&sub, z, &start, opts);
                (nfev, njev) = (nfev + info.nfev, njev + info.njev);
                // a drawing that says no bound has no root to search for: its block stands
                if bounded {
                    let s = search.get_or_insert_with(|| Search::new(self, sk));
                    let decided = s.deciding(&b.cols, &solved);
                    if !(ok && s.holds(self, z, &decided)) {
                        // the root the words say, or else what the block's own start came to
                        let first: Vec<f64> = b.cols.iter().map(|&c| z[c]).collect();
                        let mut found = false;
                        let mut attempts = s.attempts(self, z, b, &start, &decided);
                        while let Some(attempt) = attempts.next(s, self, &solved) {
                            let (mut fine, mut info) = self.solve_block(&sub, z, &attempt, opts);
                            if !fine {
                                (fine, info) = self.follow_block(&sub, z, &attempt, opts);
                            }
                            (nfev, njev) = (nfev + info.nfev, njev + info.njev);
                            if fine && s.holds(self, z, &decided) {
                                (ok, found) = (true, true);
                                break;
                            }
                        }
                        if !found {
                            for (k, &c) in b.cols.iter().enumerate() {
                                z[c] = first[k];
                            }
                        }
                    }
                }
                if !ok {
                    break;
                }
                solved.extend(b.cols.iter().copied());
            }
            for (b, consts) in self.blocks.iter_mut().zip(compiled) {
                b.consts = consts;
            }
        }
        let (mut res, rel) = self.minimise(z, Method::DogLeg, opts);
        if nfev > 0 {
            res.method = "blocks".into();
            res.nfev += nfev;
            res.njev += njev;
        }
        (res, rel)
    }

    /// One block (`sub`, its rows and columns) minimised from `start` (its own columns), every
    /// other column held at `z`, which it is written back into: whether its rows meet
    /// `acceptance_tol`, and the minimisation's count.
    fn solve_block(
        &mut self,
        sub: &Subset,
        z: &mut [f64],
        start: &[f64],
        opts: &SolveOpts,
    ) -> (bool, Info) {
        let (tol, max_nfev) = self.block_tol(opts);
        let mut zb = start.to_vec();
        let mut t = BlockTr::new(self, sub.clone(), z);
        let mut r = vec![0.0; t.m()];
        t.residuals_into(&zb, &mut r);
        let info = newton::dogleg(&mut t, &mut zb, &mut r, tol, opts.max_iter, max_nfev);
        (Self::block_back(sub, z, &zb, &r, opts), info)
    }

    /// One block followed from `start` to a root along the Newton homotopy `r(z) = (1 − t)
    /// r(start)`, `t` from 0 to 1 in `HOMOTOPY_STEPS`, each step a DogLeg from the last: where
    /// the DogLeg from a start far from every root runs off, the path keeps to the root the start
    /// leads to.  Written back into `z` as `solve_block` does.
    fn follow_block(
        &mut self,
        sub: &Subset,
        z: &mut [f64],
        start: &[f64],
        opts: &SolveOpts,
    ) -> (bool, Info) {
        let (tol, max_nfev) = self.block_tol(opts);
        let mut zb = start.to_vec();
        let inner = BlockTr::new(self, sub.clone(), z);
        let mut path = Path { inner, r0: Vec::new(), left: 1.0 };
        let mut r = vec![0.0; path.m()];
        path.inner.residuals_into(&zb, &mut r);
        path.r0 = r.clone();
        let mut total = Info { status: 0, nfev: 0, njev: 0, iterations: 0, rank: -1 };
        for k in 1..=HOMOTOPY_STEPS {
            path.left = 1.0 - k as f64 / HOMOTOPY_STEPS as f64;
            path.residuals_into(&zb, &mut r);
            let last = k == HOMOTOPY_STEPS;
            let info = newton::dogleg(&mut path, &mut zb, &mut r, tol, opts.max_iter, max_nfev);
            total.nfev += info.nfev;
            total.njev += info.njev;
            total.status = info.status;
            if last || !r.iter().all(|v| v.is_finite()) {
                break;
            }
        }
        (Self::block_back(sub, z, &zb, &r, opts), total)
    }

    /// A block's minimisation's tolerances and evaluation budget.
    fn block_tol(&self, opts: &SolveOpts) -> (newton::Tol, i32) {
        let gtol = 1e-16 / self.extent.max(1.0);
        let max_nfev = if opts.max_nfev <= 0 { 4 * opts.max_iter } else { opts.max_nfev };
        (newton::Tol { ftol: opts.tol, xtol: BLOCK_XTOL, gtol }, max_nfev)
    }

    /// A block's columns `zb` written back into `z`, and whether its rows `r` (the residuals at
    /// `zb`) meet `acceptance_tol`; NaN fails the comparison.
    fn block_back(sub: &Subset, z: &mut [f64], zb: &[f64], r: &[f64], opts: &SolveOpts) -> bool {
        for (&c, &v) in sub.cols.iter().zip(zb) {
            z[c] = v;
        }
        r.iter().all(|v| v.abs() < opts.acceptance_tol)
    }
}

/// How many steps a block followed along its Newton homotopy takes (`System::follow_block`).
const HOMOTOPY_STEPS: usize = 16;

/// A block's rows less `left` of what they were at the path's start (`System::follow_block`).
struct Path<'a> {
    inner: BlockTr<'a>,
    r0: Vec<f64>,
    left: f64,
}

impl TrustRegion for Path<'_> {
    fn n(&self) -> usize {
        self.inner.n()
    }
    fn m(&self) -> usize {
        self.inner.m()
    }
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
        self.inner.residuals_into(z, out);
        for (o, r) in out.iter_mut().zip(&self.r0) {
            *o -= self.left * r;
        }
    }
    fn jacobian_at(&mut self, z: &[f64]) {
        self.inner.jacobian_at(z)
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.inner.jt_mul(v, out)
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.inner.j_mul(v, out)
    }
    fn gn_step(&mut self, r: &[f64], g: &[f64], p: &mut [f64]) {
        self.inner.gn_step(r, g, p)
    }
    fn rank(&self) -> i32 {
        self.inner.rank()
    }
}

/// How many starts a block's root search scatters its points from (`Search::attempts`), after
/// the crossings: enough that one lands in each basin a block of a few points has.
const RESTARTS: usize = 48;

/// **A block's root, as the words say it** (§9.6, #217): what the block pass reads to choose among
/// a block's roots.  Every bound, by the columns it reads, so a block is held to the bounds it
/// decides — each column it reads among those solved so far, and some the block's own; and every
/// point and axis the solve moves, by its columns, so a block whose own start comes to no root, or
/// to one a bound refuses, starts again elsewhere: first carried across each bound it breaks as a
/// steered solve is (`across`), then with its points scattered about what the blocks before it
/// placed, at their size, its axes turned (`attempts`).  Deterministic, a fixed stream per start:
/// the drawing chooses, and not a draw.  Read once a pass, off a scratch copy of the sketch set to
/// the pass's pose where a bound or a place is read.
struct Search {
    sk: Sketch,
    free: std::collections::BTreeSet<u32>,
    /// Each bound: its constraint, and the columns its reading reads.
    bounds: Vec<(usize, Vec<usize>)>,
    /// Each point the solve moves: the point, and its own columns with its lift's.
    points: Vec<(usize, Vec<usize>)>,
    /// Each axis the solve turns: its direction's columns.
    axes: Vec<[usize; 3]>,
}

impl Search {
    fn new(sys: &System, sk: &Sketch) -> Search {
        let col = |p: u32| usize::try_from(sys.col_of[p as usize]).ok();
        let cols = |ps: Vec<u32>| {
            let mut c: Vec<usize> = ps.into_iter().filter_map(col).collect();
            c.sort_unstable();
            c.dedup();
            c
        };
        let bounds = sk.constraints.iter().enumerate()
            .filter(|(_, c)| c.bound.is_some())
            .map(|(i, c)| (i, cols(c.params(sk))))
            .filter(|(_, c)| !c.is_empty())
            .collect();
        let points = (0..sk.points.len()).filter_map(|p| {
            let mut own = sk.point_all_params(p);
            if let Some(k) = sk.lift_of(p) {
                own.extend(sk.lifts[k].x);
            }
            let c = cols(own);
            (!c.is_empty()).then_some((p, c))
        }).collect();
        let axes = sk.axes.iter().filter_map(|a| {
            let d = a.d.map(col);
            d.iter().all(Option::is_some).then(|| d.map(Option::unwrap))
        }).collect();
        Search { sk: sk.clone(), free: movable(sk), bounds, points, axes }
    }

    /// The bounds block `cols` decides, given the columns `solved` before it.
    fn deciding(&self, cols: &[usize], solved: &std::collections::BTreeSet<usize>) -> Vec<usize> {
        let own = |c: &usize| cols.binary_search(c).is_ok();
        self.bounds.iter()
            .filter(|(_, read)| {
                read.iter().any(own) && read.iter().all(|c| own(c) || solved.contains(c))
            })
            .map(|&(i, _)| i)
            .collect()
    }

    /// Whether each of the bounds `decided` holds at the pose `z`.
    fn holds(&mut self, sys: &System, z: &[f64], decided: &[usize]) -> bool {
        if decided.is_empty() {
            return true;
        }
        self.sk.set_x(&sys.full_x(z));
        let tol = bound_tol(&self.sk);
        decided.iter().all(|&ci| {
            bound_reading(&self.sk, &self.sk.constraints[ci], tol).is_none_or(|r| r.holds)
        })
    }

    /// Point `p` put at `w` in space on the scratch sketch — a point drawn in a plane at its
    /// shadow there — its lift with it; only its free numbers are written.
    fn place(&mut self, p: usize, w: [f64; 3]) {
        let at = if self.sk.points[p].z.is_some() { w.to_vec() } else {
            let (u, v) = self.sk.on_view_sheet(w, self.sk.plane_of(p));
            vec![u, v]
        };
        for (i, v) in self.sk.point_all_params(p).into_iter().zip(at) {
            if self.free.contains(&i) {
                self.sk.params[i as usize].value = v;
            }
        }
        if let Some(l) = self.sk.lift_of(p) {
            let w = self.sk.world_point(p);
            for (i, v) in self.sk.lifts[l].x.into_iter().zip(w) {
                self.sk.params[i as usize].value = v;
            }
        }
    }

    /// The block's columns as the scratch sketch has them.
    fn block_of(&self, sys: &System, b: &SolveBlock) -> Vec<f64> {
        let value = |c: usize| self.sk.params[sys.free[c] as usize].value * sys.col_scale[c];
        b.cols.iter().map(|&c| value(c)).collect()
    }

    /// Where block `b` may start again, its own start (`start`, with `z` the block's solution)
    /// having come to no root or to one a bound it decides refuses: carried across each such bound
    /// from its solution and from its start, then scattered — each made only when the one before
    /// it was not kept (`Attempts::next`).
    fn attempts<'b>(
        &mut self,
        sys: &System,
        z: &[f64],
        b: &'b SolveBlock,
        start: &[f64],
        decided: &[usize],
    ) -> Attempts<'b> {
        let mut from_start = z.to_vec();
        for (k, &c) in b.cols.iter().enumerate() {
            from_start[c] = start[k];
        }
        let x_start = sys.full_x(&from_start);
        let x_solved = sys.full_x(z);
        // across each bound the solution breaks
        self.sk.set_x(&x_solved);
        let tol = bound_tol(&self.sk);
        let broken: Vec<(usize, f64)> = decided.iter().filter_map(|&ci| {
            let r = bound_reading(&self.sk, &self.sk.constraints[ci], tol)?;
            (!r.holds).then(|| (ci, r.aims[0]))
        }).collect();
        let own = |c: &usize| b.cols.binary_search(c).is_ok();
        let moving: Vec<usize> =
            self.points.iter().filter(|(_, c)| c.iter().any(own)).map(|&(p, _)| p).collect();
        let turning: Vec<[usize; 3]> =
            self.axes.iter().filter(|d| d.iter().any(own)).copied().collect();
        Attempts {
            b,
            x_start,
            x_solved,
            broken,
            moving,
            turning,
            crossing: 0,
            mirrored: None,
            scatter: None,
            restart: 0,
        }
    }
}

/// The starts a block's root is searched from (`Search::attempts`), in order: each bound the
/// block's solution breaks crossed from the solution, then from its start — each crossing
/// followed by the block mirrored across the plane its point was carried through — and then
/// `RESTARTS` starts scattered about what the blocks before it placed.  Made one at a time, off
/// the poses taken where the search began, so a root found early costs none of the rest.
struct Attempts<'b> {
    b: &'b SolveBlock,
    x_start: Vec<f64>,
    x_solved: Vec<f64>,
    /// Each bound the block's solution breaks, with where its point aims.
    broken: Vec<(usize, f64)>,
    /// The points the block moves and the axes it turns.
    moving: Vec<usize>,
    turning: Vec<[usize; 3]>,
    /// The next crossing: the solution's for each broken bound, then the start's.
    crossing: usize,
    /// The block mirrored, made with the crossing before it and tried after it.
    mirrored: Option<Vec<f64>>,
    /// The scattered starts' centre and size, once read: `None` before, `Some(None)` where the
    /// block has none.
    scatter: Option<Option<([f64; 3], f64)>>,
    /// The next scattered start.
    restart: usize,
}

impl Attempts<'_> {
    /// The next start, or `None` where every one has been tried; `solved` the columns the blocks
    /// before placed.
    fn next(
        &mut self,
        s: &mut Search,
        sys: &System,
        solved: &std::collections::BTreeSet<usize>,
    ) -> Option<Vec<f64>> {
        let b = self.b;
        if let Some(m) = self.mirrored.take() {
            return Some(m);
        }
        let n = self.broken.len();
        while self.crossing < 2 * n {
            let x = if self.crossing < n { &self.x_solved } else { &self.x_start };
            let (ci, aim) = self.broken[self.crossing % n];
            self.crossing += 1;
            s.sk.set_x(x);
            let before: Vec<[f64; 3]> =
                self.moving.iter().map(|&p| s.sk.world_point(p)).collect();
            let Some(q) = across(&mut s.sk, ci, aim, &s.free) else { continue };
            let crossed = s.block_of(sys, b);
            // and the block mirrored across the plane the point was carried through: the other
            // hand of what it is, where the point alone comes back to where it was
            self.mirrored = self.mirror(s, sys, x, q, &before);
            return Some(crossed);
        }
        // scattered: its points about what the blocks before it placed, at that size
        let scatter = match self.scatter {
            Some(scatter) => scatter,
            None => {
                let none = (self.moving.is_empty() && self.turning.is_empty())
                    || b.cols.len() > DENSE_MAX;
                let spread = (!none).then(|| Self::spread(s, sys, &self.x_solved, solved));
                *self.scatter.insert(spread)
            }
        };
        let (centre, size) = scatter?;
        if self.restart >= RESTARTS {
            return None;
        }
        let k = self.restart;
        self.restart += 1;
        let seed = 0x2170_0000 ^ (k as u32).wrapping_mul(2_654_435_761);
        let mut rng = crate::rng::Rng::new(seed);
        s.sk.set_x(&self.x_start);
        for &p in &self.moving {
            let w = [0, 1, 2].map(|i| centre[i] + size * rng.uniform(-1.0, 1.0));
            s.place(p, w);
        }
        for d in &self.turning {
            let v = [0, 1, 2].map(|_| rng.uniform(-1.0, 1.0));
            let v = space::normalised(v).unwrap_or([0.0, 0.0, 1.0]);
            for (&c, x) in d.iter().zip(v) {
                s.sk.params[sys.free[c] as usize].value = x;
            }
        }
        Some(s.block_of(sys, b))
    }

    /// The block mirrored across the plane point `q` was carried through from `before` (the
    /// moving points where they stood at pose `x`), its axes turned with it; `None` where `q` is
    /// none of the block's or did not move.
    fn mirror(
        &self,
        s: &mut Search,
        sys: &System,
        x: &[f64],
        q: usize,
        before: &[[f64; 3]],
    ) -> Option<Vec<f64>> {
        let k = self.moving.iter().position(|&p| p == q)?;
        let (from, to) = (before[k], s.sk.world_point(q));
        let n = space::normalised(space::sub(to, from))?;
        let mid = space::lerp(from, to, 0.5);
        let mirror =
            |w: [f64; 3]| space::sub(w, space::scale(n, 2.0 * space::dot(space::sub(w, mid), n)));
        s.sk.set_x(x);
        for (&p, &w) in self.moving.iter().zip(before) {
            s.place(p, mirror(w));
        }
        for d in &self.turning {
            let v = d.map(|c| s.sk.params[sys.free[c] as usize].value);
            let v = space::sub(v, space::scale(n, 2.0 * space::dot(v, n)));
            for (&c, x) in d.iter().zip(v) {
                s.sk.params[sys.free[c] as usize].value = x;
            }
        }
        Some(s.block_of(sys, self.b))
    }

    /// Where the scattered starts are spread: the centre and size of the points the blocks before
    /// placed (`solved`), at the block's solution `x_solved`; the drawing's extent where they
    /// have none.
    fn spread(
        s: &mut Search,
        sys: &System,
        x_solved: &[f64],
        solved: &std::collections::BTreeSet<usize>,
    ) -> ([f64; 3], f64) {
        s.sk.set_x(x_solved);
        let placed: Vec<[f64; 3]> = s.points.iter()
            .filter(|(_, c)| c.iter().all(|c| solved.contains(c)))
            .map(|&(p, _)| s.sk.world_point(p))
            .collect();
        let n = placed.len().max(1) as f64;
        let centre = space::scale(placed.iter().fold([0.0; 3], |a, &w| space::add(a, w)), 1.0 / n);
        let size =
            placed.iter().map(|&w| space::norm(space::sub(w, centre))).fold(0.0, f64::max);
        (centre, if size > 0.0 { size } else { sys.extent.max(1.0) })
    }
}

/// One block of the block-triangular order as a `TrustRegion`: only its rows, only its columns,
/// every other column read from the whole free vector `z` it was handed and held there.  Rows
/// and columns keep the whole system's units (`System::subset`), so the block is minimised in
/// the same dimensionless measure the whole solve uses, and no `System` is compiled for it —
/// a compile forgets every traced contact's remembered pose.  The Gauss–Newton step is the
/// whole system's by size: minimum norm on a dense matrix up to `DENSE_MAX` columns, the
/// regularized normal equations above.
struct BlockTr<'a> {
    sys: &'a mut System,
    sub: Subset,
    z: Vec<f64>,
    dense: Option<Mat>,
    ata: Option<Ata>,
    rank: i32,
}

impl<'a> BlockTr<'a> {
    fn new(sys: &'a mut System, sub: Subset, z: &[f64]) -> BlockTr<'a> {
        let (m, n) = (sub.rows.len(), sub.cols.len());
        let dense = (n <= DENSE_MAX).then(|| Mat::zeros(m, n));
        BlockTr { sys, sub, z: z.to_vec(), dense, ata: None, rank: -1 }
    }

    /// The whole free vector with the block's own columns at `zb`.
    fn put(&mut self, zb: &[f64]) {
        for (k, &c) in self.sub.cols.iter().enumerate() {
            self.z[c] = zb[k];
        }
    }
}

impl TrustRegion for BlockTr<'_> {
    fn n(&self) -> usize {
        self.sub.cols.len()
    }
    fn m(&self) -> usize {
        self.sub.rows.len()
    }
    fn residuals_into(&mut self, zb: &[f64], out: &mut [f64]) {
        self.put(zb);
        self.sys.subset_residuals(&mut self.sub, &self.z, out);
    }
    fn jacobian_at(&mut self, zb: &[f64]) {
        self.put(zb);
        self.sys.subset_csr(&mut self.sub, &self.z);
        if let Some(j) = &mut self.dense {
            let n = j.cols;
            j.data.iter_mut().for_each(|v| *v = 0.0);
            for r in 0..j.rows {
                for p in self.sub.indptr[r]..self.sub.indptr[r + 1] {
                    let p = p as usize;
                    j.data[r * n + self.sub.indices[p] as usize] = self.sub.data[p];
                }
            }
        }
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.sub.jt_mul(v, out);
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.sub.j_mul(v, out);
    }
    fn gn_step(&mut self, r: &[f64], g: &[f64], p: &mut [f64]) {
        if let Some(j) = &self.dense {
            self.rank = newton::min_norm_step(j, r, p);
            return;
        }
        let s = &self.sub;
        let ata = self
            .ata
            .get_or_insert_with(|| Ata::new(s.rows.len(), s.cols.len(), &s.indptr, &s.indices));
        newton::normal_step(ata, &s.data, g, p);
    }
    fn rank(&self) -> i32 {
        self.rank
    }
}
