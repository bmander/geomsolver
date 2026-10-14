//! The block-triangular solve (docs/block-triangular-solve-plan.md): each strongly
//! connected block of the equations minimised in order, its root searched for where it
//! comes to none or to one a bound it decides refuses (`Search`), then a whole-system
//! polish; and the rescue of a whole-system DogLeg that did not settle by it.

use crate::linalg::Mat;
use crate::model::Sketch;
use crate::newton::{self, Info, Method, TrustRegion};
use crate::space;
use crate::sparse::Ata;
use crate::system::{SolveBlock, Subset, System, Tearing, DENSE_MAX};

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
        // the compiled constants, where the polish runs in the pass's own
        let mut kept: Vec<Vec<f64>> = Vec::new();
        if order.blocks.len() >= 2 {
            // a constant read off the pose — the directions across a line its rows are taken
            // along — is read where the blocks before put it, not where it was seeded: a line
            // turned far from its seed admits a root no word says (#217).  A pass that comes
            // through every block is polished in the frames its root was read in (in the
            // compiled ones a point it holds on a line turned since is off it by the turn); the
            // compiled ones are put back where it does not, or the polish fails, so a pass not
            // kept leaves the system as it found it
            let framed = reads_pose(sk);
            let compiled: Vec<Vec<f64>> = if framed {
                self.blocks.iter().map(|b| b.consts.clone()).collect()
            } else {
                Vec::new()
            };
            let mut pass = Pass {
                framed,
                bounded: sk.constraints.iter().any(|c| c.bound.is_some()),
                contacts: holds_contacts(sk),
                search: None,
                solved: std::collections::BTreeSet::new(),
                nfev: 0,
                njev: 0,
            };
            let mut whole = true;
            for b in &order.blocks {
                // a block finding an unknown number from a contact on an envelope or a meshing is
                // torn at it first: whole, it can come to a root on another sheet of the envelope
                // that every word it states allows
                let ok = if pass.contacts && self.finds_numbers(sk, b) {
                    self.torn(sk, &mut pass, z, b, opts)
                        || self.pass_block(sk, &mut pass, z, b, opts)
                } else {
                    self.pass_block(sk, &mut pass, z, b, opts)
                        || self.torn(sk, &mut pass, z, b, opts)
                };
                if !ok {
                    whole = false;
                    break;
                }
                pass.solved.extend(b.cols.iter().copied());
            }
            (nfev, njev) = (pass.nfev, pass.njev);
            if whole {
                kept = compiled;
            } else {
                for (b, consts) in self.blocks.iter_mut().zip(compiled) {
                    b.consts = consts;
                }
            }
        }
        let (mut res, rel) = self.minimise(z, Method::DogLeg, opts);
        if !res.success {
            for (b, consts) in self.blocks.iter_mut().zip(kept) {
                b.consts = consts;
            }
        }
        if nfev > 0 {
            res.method = "blocks".into();
            res.nfev += nfev;
            res.njev += njev;
        }
        (res, rel)
    }

    /// One block of the pass: minimised from where `z` has it, its contacts seeded first where
    /// the pass holds any, and its root searched for where it comes to none or to one a bound it
    /// decides refuses (`Search`).  Whether it came out; `z` holds what it came to, or its own
    /// start's minimisation where no search found better.
    fn pass_block(
        &mut self,
        sk: &Sketch,
        p: &mut Pass,
        z: &mut [f64],
        b: &SolveBlock,
        opts: &SolveOpts,
    ) -> bool {
        if p.framed || p.contacts {
            let s = p.search.get_or_insert_with(|| Search::new(self, sk));
            s.sk.set_x(&self.full_x(z));
            if p.framed {
                self.refresh_consts(&s.sk);
            }
            // a contact on an envelope or a meshing starts where its sources now stand, its point
            // within the bounds the block decides
            if p.contacts {
                if p.bounded {
                    s.start_within(self, z, b, &p.solved);
                }
                self.seed_contacts(&mut s.sk, z, &b.cols);
            }
        }
        let mut sub = self.subset(&b.instances, &b.cols);
        let start: Vec<f64> = b.cols.iter().map(|&c| z[c]).collect();
        let (mut ok, info) = self.solve_block(&sub, z, &start, opts);
        (p.nfev, p.njev) = (p.nfev + info.nfev, p.njev + info.njev);
        // constants read off the pose are read again where the block came out, and it is solved
        // on where its rows no longer hold there: a frame read at a start far from the drawing
        // admits a root of the frame's and none of the drawing's (`solve::REFRAMES`)
        if p.framed && ok {
            for _ in 0..super::REFRAMES {
                let s = p.search.get_or_insert_with(|| Search::new(self, sk));
                s.sk.set_x(&self.full_x(z));
                self.refresh_consts(&s.sk);
                let mut r = vec![0.0; sub.rows.len()];
                self.subset_residuals(&mut sub, z, &mut r);
                if r.iter().all(|v| v.abs() < opts.acceptance_tol) {
                    break;
                }
                let from: Vec<f64> = b.cols.iter().map(|&c| z[c]).collect();
                let (again, info) = self.solve_block(&sub, z, &from, opts);
                (p.nfev, p.njev) = (p.nfev + info.nfev, p.njev + info.njev);
                ok = again;
                if !ok {
                    break;
                }
            }
        }
        // a drawing that says no bound has no root to search for: its block stands
        if !p.bounded {
            return ok;
        }
        let s = p.search.get_or_insert_with(|| Search::new(self, sk));
        let decided = s.deciding(&b.cols, &p.solved);
        if ok && s.holds(self, z, &decided) {
            return true;
        }
        // the root the words say, or else what the block's own start came to
        let first: Vec<f64> = b.cols.iter().map(|&c| z[c]).collect();
        let mut attempts = s.attempts(self, z, b, &start, &decided);
        while let Some(mut attempt) = attempts.next(s, self, &p.solved) {
            if p.contacts {
                self.seed_attempt(s, z, &b.cols, &mut attempt);
            }
            let (mut fine, mut info) = self.solve_block(&sub, z, &attempt, opts);
            if !fine {
                (fine, info) = self.follow_block(&sub, z, &attempt, opts);
            }
            (p.nfev, p.njev) = (p.nfev + info.nfev, p.njev + info.njev);
            if fine && s.holds(self, z, &decided) {
                return true;
            }
        }
        for (k, &c) in b.cols.iter().enumerate() {
            z[c] = first[k];
        }
        false
    }

    /// **A block is torn at its unknowns** (#283): a number the drawing declares unknown
    /// (`param w: Length`, a formal a call leaves out) that a block cannot settle together with
    /// the geometry it makes — a cutter's lean found by a balance of curvatures its envelopes
    /// have — is found as a designer would find it.  Each value it is tried at, the rest of the
    /// block is solved block by block from where it stood, its contacts seeded and its roots
    /// searched for as the pass's are (`pass_block`), swept until it is still; the rows left
    /// over (`tearing`) are brought to zero by damped Newton steps over the unknowns alone,
    /// their derivatives by differences.  Tried first where the block finds a number from a
    /// contact on an envelope, and otherwise only where the block comes to no root whole.  At
    /// most `TEAR_MAX` unknowns; whether it came out, `z` holding what it came to — or as it
    /// was, where it did not.
    fn torn(
        &mut self,
        sk: &Sketch,
        p: &mut Pass,
        z: &mut [f64],
        b: &SolveBlock,
        opts: &SolveOpts,
    ) -> bool {
        let held = self.seeded_numbers(sk, b);
        if held.is_empty() || held.len() > TEAR_MAX || held.len() == b.cols.len() {
            return false;
        }
        let Some(tear) = self.tearing(b, &held) else { return false };
        let before = z.to_vec();
        let mut solved = p.solved.clone();
        solved.extend(held.iter().copied());
        let k = held.len();
        let mut out = self.subset(&tear.outer, &held);
        // the rows left over with the unknowns at `x`, the rest solved in order; `None` where a
        // part comes to no root
        let mut at = |sys: &mut System, z: &mut [f64], x: &[f64], p: &mut Pass|
            -> Option<Vec<f64>> {
            for (&c, &v) in held.iter().zip(x) {
                z[c] = v;
            }
            let outer = std::mem::replace(&mut p.solved, solved.clone());
            let mut fine = true;
            // swept until it stands still: a constant read off the pose (a plane's axes, read by
            // the points drawn in it) is a dependency the structure does not see, so a part can
            // read one a later part moves
            for _ in 0..TEAR_SWEEPS {
                let was: Vec<f64> = z.to_vec();
                p.solved = solved.clone();
                for inner in &tear.inner {
                    if !sys.pass_block(sk, p, z, inner, opts) {
                        fine = false;
                        break;
                    }
                    p.solved.extend(inner.cols.iter().copied());
                }
                let still =
                    z.iter().zip(&was).all(|(a, b)| (a - b).abs() <= 1e-12 * a.abs().max(1.0));
                if !fine || still {
                    break;
                }
            }
            p.solved = outer;
            if !fine {
                return None;
            }
            let mut r = vec![0.0; out.rows.len()];
            sys.subset_residuals(&mut out, z, &mut r);
            r.iter().all(|v| v.is_finite()).then_some(r)
        };
        let norm = |r: &[f64]| r.iter().map(|v| v * v).sum::<f64>().sqrt();
        let mut x: Vec<f64> = held.iter().map(|&c| z[c]).collect();
        let Some(mut r) = at(self, z, &x, p) else {
            z.copy_from_slice(&before);
            return false;
        };
        let mut done = r.iter().all(|v| v.abs() < opts.acceptance_tol);
        for _ in 0..TEAR_STEPS {
            if done {
                break;
            }
            // the derivative of the rows left over along each unknown, by differences from here
            let here = z.to_vec();
            let mut jac = Mat::zeros(r.len(), k);
            let mut fine = true;
            for j in 0..k {
                let h = TEAR_DIFF * x[j].abs().max(1.0);
                let mut xh = x.clone();
                xh[j] += h;
                match at(self, z, &xh, p) {
                    Some(rh) => {
                        for i in 0..r.len() {
                            jac.data[i * k + j] = (rh[i] - r[i]) / h;
                        }
                    }
                    None => fine = false,
                }
                z.copy_from_slice(&here);
            }
            if !fine {
                break;
            }
            let mut step = vec![0.0; k];
            crate::newton::min_norm_step(&jac, &r, &mut step);
            // halved until the rows left over come down, each try from where the last stood
            let mut taken = false;
            let mut scale = 1.0;
            for _ in 0..TEAR_HALVINGS {
                let xt: Vec<f64> = x.iter().zip(&step).map(|(a, d)| a + scale * d).collect();
                let got = at(self, z, &xt, p);
                if let Some(rt) = got {
                    if norm(&rt) < norm(&r) {
                        (x, r, taken) = (xt, rt, true);
                        break;
                    }
                }
                z.copy_from_slice(&here);
                scale *= 0.5;
            }
            if !taken {
                break;
            }
            done = r.iter().all(|v| v.abs() < opts.acceptance_tol);
        }
        if !done {
            z.copy_from_slice(&before);
        }
        done
    }

    /// The unknown numbers block `b` is torn at (`torn`): those the drawing declares with a
    /// seed — `param w: Length hint(80mm)`, a formal a call leaves out seeded `hint(…)`, where
    /// the designer says where the search for one starts — or, where the block holds none, every
    /// unknown number it holds.
    fn seeded_numbers(&self, sk: &Sketch, b: &SolveBlock) -> Vec<usize> {
        let col = |q: u32| col_among(&self.col_of, &b.cols, q);
        let numbers: Vec<(usize, bool)> = sk.free_vars.iter()
            .filter_map(|(name, &q)| {
                let seeded = sk.declared.get(name).is_some_and(|d| d.seed.is_some());
                col(q).map(|c| (c, seeded))
            })
            .collect();
        let seeded: Vec<usize> = numbers.iter().filter(|n| n.1).map(|n| n.0).collect();
        let mut held =
            if seeded.is_empty() { numbers.iter().map(|n| n.0).collect() } else { seeded };
        held.sort_unstable();
        held
    }

    /// Whether block `b` holds both an unknown number the drawing declares and the place of a
    /// contact on an envelope or a meshing.
    fn finds_numbers(&self, sk: &Sketch, b: &SolveBlock) -> bool {
        use crate::constraints::Arg;
        let col = |q: u32| col_among(&self.col_of, &b.cols, q).is_some();
        sk.free_vars.values().any(|&q| col(q))
            && sk.constraints.iter().any(|c| {
                is_contact(c.kind) && c.args.iter().any(|a| matches!(a, Arg::Param(q) if col(*q)))
            })
    }

    /// Where block `b` is torn at its unknowns `held` (`torn`): the instances whose rows are left
    /// over — one or two statements whose rows number the unknowns, and without which the rest
    /// is square and perfectly matched — and the rest in its own block-triangular order.  Among
    /// the choices, the one whose widest inner block is smallest, the latest statements first
    /// among equals.  `None` where no such statements are found.  A fact about the compile's
    /// structure, so worked out once a compile (`System::tearings`): a solve steered by a bound
    /// passes the blocks again.
    fn tearing(&mut self, b: &SolveBlock, held: &[usize]) -> Option<std::sync::Arc<Tearing>> {
        let key = (b.cols.clone(), held.to_vec());
        if let Some(t) = self.tearings.get(&key) {
            return t.clone();
        }
        let t = self.tear(b, held).map(std::sync::Arc::new);
        self.tearings.insert(key, t.clone());
        t
    }

    /// `tearing`, worked out: every choice of statements tried, the latest first.  Each row's
    /// columns among the rest are read once, a choice that leaves a column no row is refused
    /// before it is matched, and the rest is matched alone — square, it is well determined
    /// exactly where the matching is perfect, which is all `graph::blocks` reads of it; the
    /// choices are compared by their widest part alone (`graph::widest_block`), and only the one
    /// chosen is put in order.
    fn tear(&self, b: &SolveBlock, held: &[usize]) -> Option<Tearing> {
        let (adj, _) = self.structure();
        let hard = self.hard_rows();
        let mut index = vec![usize::MAX; self.n_res];
        for (i, &r) in hard.iter().enumerate() {
            index[r] = i;
        }
        let rest: Vec<usize> = b.cols.iter().copied().filter(|c| !held.contains(c)).collect();
        let n = rest.len();
        let mut local = vec![usize::MAX; self.n_free];
        for (i, &c) in rest.iter().enumerate() {
            local[c] = i;
        }
        // each instance's rows, and each row's columns among the rest, read once
        let rows_of: Vec<Vec<usize>> =
            b.instances.iter().map(|i| self.rows_of(std::slice::from_ref(i))).collect();
        let mut cols_of = vec![Vec::new(); self.n_res];
        let mut cover = vec![0_usize; n];
        for &r in rows_of.iter().flatten() {
            cols_of[r] = adj[index[r]].iter().map(|&c| local[c]).filter(|&c| c != usize::MAX)
                .collect::<Vec<usize>>();
            for &c in &cols_of[r] {
                cover[c] += 1;
            }
        }
        let id = |k: usize| self.blocks[b.instances[k].0].cids[b.instances[k].1];
        // the candidates, latest statement first, by their place in `b.instances`
        let mut by_id: Vec<usize> = (0..b.instances.len()).collect();
        by_id.sort_by_key(|&k| std::cmp::Reverse(id(k)));
        let need = held.len();
        let mut uncovered = cover.clone();
        // the rest without the statements `outer`, matched: its rows, each row's columns among
        // the rest, and the matching; `None` where it is not square and perfectly matched
        let mut matched = |outer: &[usize]| -> Option<(Vec<usize>, Vec<Vec<usize>>, crate::graph::Dm)> {
            if outer.iter().map(|&k| rows_of[k].len()).sum::<usize>() != need {
                return None;
            }
            // a column only the rows left over read leaves the rest short of a row for it
            uncovered.copy_from_slice(&cover);
            for &r in outer.iter().flat_map(|&k| &rows_of[k]) {
                for &c in &cols_of[r] {
                    uncovered[c] -= 1;
                }
            }
            if uncovered.contains(&0) {
                return None;
            }
            let rows: Vec<usize> = (0..b.instances.len()).filter(|k| !outer.contains(k))
                .flat_map(|k| rows_of[k].iter().copied()).collect();
            if rows.len() != n {
                return None;
            }
            let sub: Vec<Vec<usize>> = rows.iter().map(|&r| cols_of[r].clone()).collect();
            let (mate_row, mate_col) = crate::graph::hopcroft_karp(&sub, n);
            if mate_row.contains(&-1) {
                return None;
            }
            // square and perfectly matched: well determined throughout
            let dm = crate::graph::Dm {
                mate_row,
                mate_col,
                over_rows: Vec::new(),
                over_cols: Vec::new(),
                under_rows: Vec::new(),
                under_cols: Vec::new(),
                well_rows: (0..n).collect(),
                well_cols: (0..n).collect(),
                rank: n,
                n_redundant: 0,
                n_free: 0,
            };
            Some((rows, sub, dm))
        };
        // of every tearing that leaves the rest square, the one whose largest part is smallest,
        // the later statements first among equals: the rows left over are those the rest of the
        // block does not need to come apart
        let mut best: Option<(Vec<usize>, usize)> = None;
        let mut consider = |outer: Vec<usize>| {
            if let Some((_, sub, dm)) = matched(&outer) {
                let widest = crate::graph::widest_block(&sub, &dm.mate_col);
                if best.as_ref().is_none_or(|b| widest < b.1) {
                    best = Some((outer, widest));
                }
            }
        };
        for (j, &a) in by_id.iter().enumerate() {
            consider(vec![a]);
            if need >= 2 {
                for &c in &by_id[j + 1..] {
                    consider(vec![a, c]);
                }
            }
        }
        // the one chosen, in its own block-triangular order
        let (outer, _) = best?;
        let (rows, sub, dm) = matched(&outer)?;
        let blt = crate::graph::blocks(&sub, &dm);
        let inner = blt.rows.iter().map(|part| {
            let mut cols: Vec<usize> = part.iter().map(|&r| rest[dm.mate_row[r] as usize]).collect();
            cols.sort_unstable();
            let mut instances: Vec<(usize, usize)> =
                part.iter().map(|&r| self.instance_of(rows[r])).collect();
            instances.sort_unstable();
            instances.dedup();
            let full = self.rows_of(&instances);
            SolveBlock { rows: full, cols, instances }
        }).collect();
        Some(Tearing { outer: outer.iter().map(|&k| b.instances[k]).collect(), inner })
    }

    /// **A contact starts where its block comes up** (#304): a point on an envelope in space or
    /// on a meshing is seeded as it is stated (`Sketch::add`), where the sources it reads stand
    /// before any relation has placed them, and a place sought on the wrong flanks leaves Newton
    /// a root it reaches only from near it.  So each contact whose own unknowns are among the
    /// block's columns `cols` is seeded again with the pose `sk` the blocks before it left —
    /// its point where it stands (its hidden point brought to it first, as the block's other
    /// hidden points are) — and `z` takes the seed.  A pinned phase or lag is kept, as
    /// `System::remeshed` keeps one.  Nothing is done for a block with no such contact.
    fn seed_contacts(&mut self, sk: &mut Sketch, z: &mut [f64], cols: &[usize]) {
        use crate::constraints::{Arg, CKind, SpecKind};
        // (its own copy: the closure outlives the residuals read below, which borrow `self`)
        let col_of = self.col_of.clone();
        let col = |q: u32| col_among(&col_of, cols, q);
        let place = |c: &crate::constraints::Constraint| -> Vec<(usize, u32)> {
            let first = match c.kind {
                CKind::PointOnEnvelope => c.kind.contact_on(SpecKind::Envelope).map(|(_, t)| t),
                CKind::PointOnMeshing => Some(3),
                _ => None,
            };
            let Some(first) = first else { return Vec::new() };
            c.args.iter().enumerate().skip(first).filter_map(|(i, a)| match a {
                Arg::Param(q) if !sk.params[*q as usize].fixed => col(*q).map(|_| (i, *q)),
                _ => None,
            }).collect()
        };
        // one near holding where it stands keeps its place, and Newton takes it from there: an
        // envelope can pass near its point on another sheet, and the place nearest it is as
        // likely to be that one; a seed taken where the sources stood before any relation placed
        // them is off by the drawing's size
        let found: Vec<(usize, Vec<(usize, u32)>)> = sk.constraints.iter().enumerate()
            .map(|(i, c)| (i, place(c)))
            .filter(|(_, slots)| !slots.is_empty())
            .collect();
        let mut contacts = Vec::new();
        for (i, slots) in found {
            // its rows as the system reads them, over their own units
            let Some(&inst) = self.slot_of.get(&sk.constraints[i].id) else { continue };
            let mut sub = self.subset(&[inst], cols);
            let mut r = vec![0.0; sub.rows.len()];
            self.subset_residuals(&mut sub, z, &mut r);
            if !r.iter().all(|v| v.abs() <= SEEDED_OFF) {
                contacts.push((i, slots));
            }
        }
        if contacts.is_empty() {
            return;
        }
        // every hidden point the block moves, where its drawn point stands: a contact reads its
        // point's, and a circle a contact holds reads its centre's
        for k in 0..sk.lifts.len() {
            let l = &sk.lifts[k];
            if !l.x.iter().any(|&q| col(q).is_some()) {
                continue;
            }
            let (x, w) = (l.x, sk.world_point(l.point as usize));
            for (q, v) in x.into_iter().zip(w) {
                if let Some(c) = col(q) {
                    sk.params[q as usize].value = v;
                    z[c] = v * self.col_scale[c];
                }
            }
        }
        for (i, slots) in contacts {
            let c = &sk.constraints[i];
            let seeds: Vec<f64> = match c.kind {
                CKind::PointOnEnvelope => {
                    slots.iter()
                        .map(|&(k, _)| crate::constraints::seed_param(sk, c.kind, &c.args, k))
                        .collect()
                }
                _ => {
                    let Arg::Ent(p) = c.args[0] else { continue };
                    let lift = sk.lifted(p.i());
                    let Some(seed) = crate::constraints::meshing_seed(sk, c, lift) else {
                        continue;
                    };
                    slots.iter().map(|&(k, _)| seed[k - 2]).collect()
                }
            };
            for (&(_, q), v) in slots.iter().zip(seeds) {
                if !v.is_finite() {
                    continue;
                }
                let c = col(q).expect("a slot among the block's columns");
                sk.params[q as usize].value = v;
                z[c] = v * self.col_scale[c];
            }
        }
    }

    /// A searched start of a block seeded as its first start is (`seed_contacts`): the
    /// attempt's columns put where the scratch sketch reads them, the contacts seeded there, and
    /// `attempt` given back with their seeds.
    fn seed_attempt(&mut self, s: &mut Search, z: &[f64], cols: &[usize], attempt: &mut [f64]) {
        let mut at = z.to_vec();
        for (k, &c) in cols.iter().enumerate() {
            at[c] = attempt[k];
        }
        s.sk.set_x(&self.full_x(&at));
        self.seed_contacts(&mut s.sk, &mut at, cols);
        for (k, &c) in cols.iter().enumerate() {
            attempt[k] = at[c];
        }
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

/// What one block pass carries from block to block (`System::pass_block`): what it asks of the
/// drawing, the scratch sketch its searches read, the columns placed so far and its count.
struct Pass {
    framed: bool,
    bounded: bool,
    contacts: bool,
    search: Option<Search>,
    solved: std::collections::BTreeSet<usize>,
    nfev: i32,
    njev: i32,
}

/// How many unknowns a block is torn at, at most (`System::torn`).
const TEAR_MAX: usize = 2;
/// How many Newton steps a torn block's unknowns take, at most.
const TEAR_STEPS: usize = 40;
/// How many times a torn block's step is halved before it is given up.
const TEAR_HALVINGS: usize = 12;
/// How many times a torn block's rest is swept, at most, for one value of its unknowns.
const TEAR_SWEEPS: usize = 4;
/// The step a torn block's derivative is taken over, relative to the unknown's size.
const TEAR_DIFF: f64 = 1e-6;
/// How far off a contact may hold, its rows over their own units, and keep its place when
/// its block comes up (`System::seed_contacts`).
const SEEDED_OFF: f64 = 1e-2;

/// Whether a drawing holds a point to an envelope in space or to a meshing (#283): a contact
/// whose place is seeded again as its block comes up (`System::seed_contacts`), so its document
/// solve takes the block pass first (`System::solve_compiled`).
pub(super) fn holds_contacts(sk: &Sketch) -> bool {
    sk.constraints.iter().any(|c| is_contact(c.kind))
}

/// Whether a constraint of kind `kind` holds a point to an envelope in space or to a meshing.
fn is_contact(kind: crate::constraints::CKind) -> bool {
    use crate::constraints::CKind;
    matches!(kind, CKind::PointOnEnvelope | CKind::PointOnMeshing)
}

/// The column parameter `q` is solved in, where that is one of `cols` (sorted).
fn col_among(col_of: &[i32], cols: &[usize], q: u32) -> Option<usize> {
    usize::try_from(col_of[q as usize]).ok().filter(|c| cols.binary_search(c).is_ok())
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

    /// A block holding a contact started within the bounds it decides (#304): its points carried
    /// across each one its start breaks, twice round — before its contacts are seeded where
    /// those points then stand (`System::seed_contacts`).  A point on an envelope that nothing
    /// seeds stands wherever the drawing scattered it, and the place nearest it is as likely on
    /// another sheet of the envelope as on the one its bounds say; carried into them first, it
    /// starts near its root.  `z` and the scratch sketch, at `z`'s pose, take the crossings.
    fn start_within(&mut self, sys: &System, z: &mut [f64], b: &SolveBlock,
        solved: &std::collections::BTreeSet<usize>) {
        let decided = self.deciding(&b.cols, solved);
        if decided.is_empty() {
            return;
        }
        let tol = bound_tol(&self.sk);
        // only the block's own points are carried: one the blocks before it placed stands
        let own: std::collections::BTreeSet<u32> =
            b.cols.iter().map(|&c| sys.free[c] as u32).filter(|q| self.free.contains(q)).collect();
        for _ in 0..2 {
            for &ci in &decided {
                let Some(r) = bound_reading(&self.sk, &self.sk.constraints[ci], tol) else {
                    continue;
                };
                if !r.holds {
                    across(&mut self.sk, ci, r.aims[0], &own);
                }
            }
        }
        let v = self.block_of(sys, b);
        for (k, &c) in b.cols.iter().enumerate() {
            z[c] = v[k];
        }
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
        let breaks = |sk: &Sketch| -> Vec<(usize, f64)> {
            decided.iter().filter_map(|&ci| {
                let r = bound_reading(sk, &sk.constraints[ci], tol)?;
                (!r.holds).then(|| (ci, r.aims[0]))
            }).collect()
        };
        let broken = breaks(&self.sk);
        // and across each the start breaks that the solution does not: a block whose own start
        // came to no root at all leaves no broken bound at its solution to carry it across
        self.sk.set_x(&x_start);
        let started: Vec<(usize, f64)> =
            breaks(&self.sk).into_iter()
                .filter(|(ci, _)| !broken.iter().any(|(b, _)| b == ci)).collect();
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
            started,
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
    /// Each bound the block's start breaks and its solution does not, with where it aims.
    started: Vec<(usize, f64)>,
    /// The points the block moves and the axes it turns.
    moving: Vec<usize>,
    turning: Vec<[usize; 3]>,
    /// The next crossing: the solution's for each broken bound, then the start's, then the start's
    /// for each bound only the start breaks.
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
        while self.crossing < 2 * n + self.started.len() {
            let x = if self.crossing < n { &self.x_solved } else { &self.x_start };
            let (ci, aim) = if self.crossing < 2 * n {
                self.broken[self.crossing % n]
            } else {
                self.started[self.crossing - 2 * n]
            };
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
