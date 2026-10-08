//! **A tangency's derivative** (§6.21, #104): the geometry a use of a set makes, paired with how it
//! moves along the set.
//!
//! `l tangent S` states the set's body at a contact on `l`, and every row of it again as its
//! derivative, `F_x·ẋ = 0`, as the contact moves along `l` (`kernels::dual_kernel`).  What the
//! derivative reads a column moving by is the column's **motion**: the contact's place in space
//! moves by the line's direction; geometry the use made privately (a point the body declares, its
//! lift) by a **tangent unknown** of its own, minted here as rows read it, so its motion is solved
//! for with everything else; and what the set was given is held.  A tangency between two sets at a
//! point (`S1 tangent(at: m) S2`) is two of them, each moving the point by a tangent unknown in a
//! chart: `e_a + s·e_c`, `c` the world axis nearest the sets' normal.
//!
//! The private geometry's own rows (a lift, an axis's unit) are stated as their derivatives too,
//! here, once both a row and the use are known — so a point the body draws in a view moves in it.

use super::*;
use crate::constraints::Constraint;
use std::collections::BTreeSet;

/// What moves a dual's point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Toward {
    /// `l tangent S`: the line's direction, `b − a`.
    Line(usize),
    /// A direction in the tangent space solved for, the `k`th of two (`S1 tangent(at: m) S2`),
    /// gauged by a chart: one along `e_a`, the other along `e_b`, each rising along `e_c` by an
    /// unknown — `axis` is `c`, once chosen (`program::views::charts`).
    Chart { k: u8, axis: Option<u8> },
}

/// One use's derivative: the point that moves, what moves it, and the geometry the use made, whose
/// motion is unknown.
#[derive(Clone, Debug, PartialEq)]
pub struct Dual {
    pub point: usize,
    pub toward: Toward,
    pub owned: Vec<EntRef>,
    /// Each moving column's tangent column.  Derived, minted as rows read them; never saved.
    pub tangent: BTreeMap<u32, u32>,
}

/// How a column moves under a dual (`kernels::dual_kernel`'s mask).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Move {
    Held,
    /// A component of the line's direction, 0 to 2.
    Dir(usize),
    /// By a tangent column of its own.
    Tangent,
}

/// The columns a dual moves, read once for a row's worth of questions.
pub struct Moves {
    point: Option<[u32; 3]>,
    toward: Toward,
    owned: BTreeSet<u32>,
}

impl Moves {
    pub fn of(&self, p: u32) -> Move {
        if let Some(k) = self.point.and_then(|x| x.iter().position(|&q| q == p)) {
            return match self.toward {
                Toward::Line(_) => Move::Dir(k),
                Toward::Chart { .. } => Move::Tangent,
            };
        }
        if self.owned.contains(&p) { Move::Tangent } else { Move::Held }
    }

    /// Whether `p` is a column of geometry the use made — whose own rows move with it.
    pub fn made(&self, p: u32) -> bool {
        self.owned.contains(&p)
    }
}

impl Sketch {
    /// A use's derivative, registered: its index, which a derivative row names
    /// (`Constraint::along`).  The private geometry's rows already stated are stated again as
    /// their derivatives.
    pub fn add_dual(&mut self, point: usize, toward: Toward, owned: Vec<EntRef>) -> usize {
        self.duals.push(Dual { point, toward, owned, tangent: BTreeMap::new() });
        let d = self.duals.len() - 1;
        let rows: Vec<usize> = (0..self.constraints.len()).collect();
        self.twin_intrinsics(d, &rows);
        d
    }

    /// Which columns dual `d` moves, and how.
    pub fn dual_moves(&self, d: usize) -> Moves {
        let dual = &self.duals[d];
        let mut owned = BTreeSet::new();
        for &e in &dual.owned {
            owned.extend(self.own_params(e));
            if e.kind == EntKind::Point {
                if let Some(k) = self.lift_of(e.i()) {
                    owned.extend(self.lifts[k].x);
                }
            }
        }
        let point = self.lift_of(dual.point).map(|k| self.lifts[k].x);
        Moves { point, toward: dual.toward, owned }
    }

    /// The tangent column of `p` under dual `d`, or the fixed zero where it has none.
    pub fn tangent_col(&self, d: usize, p: u32) -> u32 {
        match self.duals[d].tangent.get(&p) {
            Some(&t) => t,
            None => self.zero.expect("a derivative row mints the zero at the add"),
        }
    }

    /// Mint the tangent columns a derivative row over `cols` reads, seeded still; a chart's are
    /// gauged as it says (`set_chart`).
    pub(crate) fn mint_tangents(&mut self, d: usize, cols: &[u32]) {
        self.origin_param();
        let moves = self.dual_moves(d);
        for &p in cols {
            if moves.of(p) != Move::Tangent || self.duals[d].tangent.contains_key(&p) {
                continue;
            }
            let name = format!("{}.d", self.params[p as usize].name);
            let t = self.param(0.0, false, &name) as u32;
            self.duals[d].tangent.insert(p, t);
        }
        if let Toward::Chart { axis: Some(c), .. } = self.duals[d].toward {
            self.set_chart(d, c);
        }
    }

    /// A chart's gauge: the point's tangent `e_a + s·e_c` (`k` = 0) or `e_b + s·e_c` (`k` = 1),
    /// `a`, `b` the world axes other than `c` in order — two of its three components held.
    pub fn set_chart(&mut self, d: usize, c: u8) {
        let Toward::Chart { k, .. } = self.duals[d].toward else { return };
        self.duals[d].toward = Toward::Chart { k, axis: Some(c) };
        let Some(x) = self.lift_of(self.duals[d].point).map(|l| self.lifts[l].x) else { return };
        let others: Vec<usize> = (0..3).filter(|&a| a != c as usize).collect();
        for (a, &col) in x.iter().enumerate() {
            let Some(&t) = self.duals[d].tangent.get(&col) else { continue };
            let p = &mut self.params[t as usize];
            match others.iter().position(|&o| o == a) {
                Some(j) => {
                    p.value = if j == k as usize { 1.0 } else { 0.0 };
                    p.fixed = true;
                }
                None => p.fixed = false,
            }
        }
    }

    /// Each chart not yet gauged, gauged where its point stands now: `c` is the world axis the
    /// sets' normal there runs most along, read off the gradients of the rows the chart's
    /// directions differentiate in the point's place — so the two directions `e_a + s·e_c`,
    /// `e_b + s·e_c` span the tangent plane wherever its normal is off square to `c`.  A choice
    /// of chart, made once: a document carries it (`set_chart`).
    pub fn choose_charts(&mut self) {
        for d in 0..self.duals.len() {
            let Toward::Chart { axis: None, .. } = self.duals[d].toward else { continue };
            let Some(x) = self.lift_of(self.duals[d].point).map(|l| self.lifts[l].x) else { continue };
            let mut score = [0.0f64; 3];
            for c in self.constraints.iter().filter(|c| c.along == Some(d)) {
                let cols = c.row_params(self);
                let v: Vec<f64> = cols.iter().map(|&p| self.params[p as usize].value).collect();
                let (_, jac) = crate::kernels::eval_one(c.kernel_id(), &v, &c.row_consts(self));
                for row in jac.chunks(cols.len().max(1)) {
                    let g: [f64; 3] = std::array::from_fn(|a| {
                        cols.iter().zip(row).filter(|(p, _)| **p == x[a]).map(|(_, g)| g).sum()
                    });
                    let n = crate::space::norm(g);
                    if n > 0.0 {
                        for a in 0..3 {
                            score[a] += (g[a] / n).abs();
                        }
                    }
                }
            }
            let c = (0..3).fold(0, |best, a| if score[a] > score[best] { a } else { best });
            self.set_chart(d, c as u8);
        }
    }

    /// The private geometry's own rows among `rows` (constraint indices), each stated again as its
    /// derivative under dual `d` where it reads a column of that geometry — and never the
    /// point's own row: a tangency at a point drawn in a view reads where it stands in space,
    /// moving in no view.
    fn twin_intrinsics(&mut self, d: usize, rows: &[usize]) {
        let moves = self.dual_moves(d);
        let twins: Vec<Constraint> = rows
            .iter()
            .map(|&i| &self.constraints[i])
            .filter(|c| c.intrinsic && c.along.is_none() && !c.soft)
            .filter(|c| c.params(self).iter().any(|&p| moves.made(p)))
            .map(|c| {
                let mut t = c.clone();
                t.id = 0;
                t.along = Some(d);
                t
            })
            .collect();
        for t in twins {
            self.add_quiet(t);
        }
    }

    /// A row just added, stated again as its derivative under every dual it is the private
    /// geometry's row of — a lift minted after the use was registered.
    pub(crate) fn twin_new_intrinsic(&mut self, at: usize) {
        if self.duals.is_empty() || !self.constraints[at].intrinsic || self.constraints[at].along.is_some() {
            return;
        }
        for d in 0..self.duals.len() {
            self.twin_intrinsics(d, &[at]);
        }
    }

    /// Retire the tangent columns of dual `d` no row reads now: a free number no equation
    /// mentions is no freedom.
    pub(crate) fn retire_tangents(&mut self, d: usize) {
        let read: BTreeSet<u32> = self
            .constraints
            .iter()
            .filter(|c| c.along == Some(d))
            .flat_map(|c| c.params(self))
            .collect();
        let unread: Vec<u32> =
            self.duals[d].tangent.values().copied().filter(|t| !read.contains(t)).collect();
        for t in unread {
            self.params[t as usize].fixed = true;
        }
    }
}
