//! A view's attitude as unknowns, and the hidden points in space a spatial relation reads
//! (`docs/spatial-constraints-plan.md`, P1).
//!
//! Nothing here is minted unless asked for: a document that states every plane — every
//! document today — has no `Att` and no `LiftE`, so its parameter vector, its constraints and
//! every number a solve reads are what they were.

use super::*;
use crate::constraints::CKind;
use crate::plane::{self, Quat};

impl Sketch {
    /// The five numbers an attitude's unknowns hold now: `q` and then `d`.
    pub(in crate::model) fn att_values(&self, a: &Att) -> [f64; 5] {
        let g = |p: u32| self.params[p as usize].value;
        [g(a.q[0]), g(a.q[1]), g(a.q[2]), g(a.q[3]), g(a.d)]
    }

    /// Plane `i`'s attitude as unknowns, and the intrinsic `quat_unit` row holding its
    /// quaternion to the unit sphere — minted once; asked again it only frees them.
    ///
    /// The unknowns start at the stated basis: `q` turns the page's axes onto `u`, `v`, `n`,
    /// `d` is the stored origin along `n`, and `ab` the rest of it in the plane, so the view
    /// stands where it stood and `basis(i)` answers with the stored basis to the bit until a
    /// solve moves it (`Att::seat`).  Net four freedoms, three turning and one along the normal;
    /// every `lift` already written on the view's points changes to the twin that reads them.
    pub fn free_attitude(&mut self, i: usize) {
        self.mint_attitude(i, true);
        self.fix_attitude(i, false);
    }

    /// Plane `i`'s attitude as unknowns **held by a hinge** to the view it is folded from
    /// (P2a): minted as `free_attitude` would but with no `quat_unit` row — the hinge's four rows
    /// say what the quaternion is — its quaternion `q`, the parent's times the fold's turn, so
    /// the hinge starts satisfied and not merely up to a sign, and its offset held (a fold turns
    /// a view about a line through its parent's origin; `through:` or `offset: free` let it go
    /// again with `fix_offset`).  The stored basis stays what a reader sees until a solve moves
    /// the view, as it does for a freed one.
    pub fn hinge_attitude(&mut self, i: usize, q: Quat) {
        self.mint_attitude(i, false);
        let a = self.planes[i].att.clone().expect("just minted");
        for k in 0..4 {
            self.params[a.q[k] as usize].value = q[k];
            self.params[a.q[k] as usize].fixed = false;
        }
        self.params[a.d as usize].fixed = true;
        let at = self.planes[i].att.as_mut().expect("present");
        at.seat[..4].copy_from_slice(&q);
    }

    /// Mint plane `i`'s unknowns once — with the `quat_unit` row when `unit`, and as a hinged
    /// view's when not — and turn every statement reading the view to the twin its unknowns
    /// feed.  A projection that turns needs both its views solved and both its images lifted,
    /// so the other view is given held unknowns of its own and the images their hidden points.
    pub(in crate::model) fn mint_attitude(&mut self, i: usize, unit: bool) {
        if self.planes[i].att.is_some() {
            return;
        }
        let name = {
            let c = &self.params[self.planes[i].frame.c as usize].name;
            c.strip_suffix(".c").unwrap_or(c).to_string()
        };
        let (q, d, ab) = seated(&self.planes[i].basis);
        // one unit of `q` turns the view's content by about its own extent
        let scale = self.att_scale(i, ab, d);
        let qp = ["w", "x", "y", "z"]
            .iter()
            .enumerate()
            .map(|(k, n)| self.param_scaled(q[k], false, &format!("{name}.q{n}"), scale) as u32)
            .collect::<Vec<_>>();
        let dp = self.param(d, false, &format!("{name}.d")) as u32;
        let seat = [q[0], q[1], q[2], q[3], d];
        self.planes[i].att =
            Some(Att { q: [qp[0], qp[1], qp[2], qp[3]], d: dp, ab, seat, hinged: !unit });
        if unit {
            let mut c = Constraint::new(CKind::QuatUnit, vec![crate::constraints::Arg::Ent(
                EntRef::plane(i),
            )]);
            c.intrinsic = true;
            self.add(c);
        }
        // a statement written against the stated basis — a lift, a point on the plane, a
        // circle drawn in it, a projection — now reads the unknowns instead: the same
        // statement, a different twin, so the constraint keeps its id and its arguments
        let flip: Vec<usize> = (0..self.constraints.len())
            .filter(|&k| self.constraints[k].attitudes_read(self).contains(&i))
            .collect();
        let mut projections = Vec::new();
        for k in flip {
            let c = &mut self.constraints[k];
            c.kind = c.kind.attitude_twin(true);
            if c.kind == CKind::ProjectSolved {
                projections.push(k);
            }
        }
        for k in projections {
            self.solve_projection(k);
        }
    }

    /// What a projection over a solved view reads that a stated one did not: the other view's
    /// quaternion — held where it is, if that view is stated — and both images' hidden points.
    pub(in crate::model) fn solve_projection(&mut self, k: usize) {
        let c = &self.constraints[k];
        let (a, b) = (c.args[0].ent().i(), c.args[1].ent().i());
        for v in [c.args[2].ent().i(), c.args[3].ent().i()] {
            if self.planes[v].att.is_none() {
                self.mint_attitude(v, true);
                self.fix_attitude(v, true);
            }
        }
        self.lift_point(a);
        self.lift_point(b);
    }

    /// Hold a solved view's quaternion, or let it go — the turn alone, its offset untouched.
    pub fn fix_turn(&mut self, i: usize, fixed: bool) {
        let Some(a) = self.planes[i].att.clone() else { return };
        for p in a.q {
            self.params[p as usize].fixed = fixed;
        }
    }

    /// Hold a solved view's offset along its normal, or let it go — `offset: free` and
    /// `through:` are the offset solved while the turn is whatever else says it is.
    pub fn fix_offset(&mut self, i: usize, fixed: bool) {
        let Some(a) = self.planes[i].att.clone() else { return };
        self.params[a.d as usize].fixed = fixed;
    }

    /// Hold plane `i`'s solved attitude where it is, or let it go again — the params' own
    /// `fixed` flags, so nothing moves and no number changes either way.  A plane with no `Att`
    /// is stated and has nothing to hold.
    pub fn fix_attitude(&mut self, i: usize, fixed: bool) {
        let Some(a) = self.planes[i].att.clone() else { return };
        for p in a.q.iter().chain([&a.d]) {
            self.params[*p as usize].fixed = fixed;
        }
    }

    /// A solved attitude as a document or a copy recorded it: minted as `free_attitude` would,
    /// then given the numbers — the quaternion, the offset, which of them are held, and the
    /// constants `ab`.  The seat stays the one the stored basis gave, so a view that had not
    /// moved when it was written reads back as the stored basis, and one that had reads its
    /// unknowns.
    pub fn restore_attitude(&mut self, i: usize, q: Quat, qfixed: bool, d: f64, dfixed: bool,
                            ab: [f64; 2], hinged: bool) {
        self.mint_attitude(i, !hinged);
        let a = self.planes[i].att.as_mut().expect("just minted");
        a.ab = ab;
        let a = a.clone();
        for k in 0..4 {
            self.params[a.q[k] as usize].value = q[k];
            self.params[a.q[k] as usize].fixed = qfixed;
        }
        self.params[a.d as usize].value = d;
        self.params[a.d as usize].fixed = dfixed;
    }

    /// Re-read a solved view's unknowns off its stored basis — what a writer of the basis does,
    /// so `set_basis` and `set_plane_origin` mean the same thing on a solved view as on a stated
    /// one.
    pub(in crate::model) fn seat_attitude(&mut self, i: usize) {
        let Some(a) = self.planes[i].att.clone() else { return };
        let (mut q, d, ab) = seated(&self.planes[i].basis);
        // the same turn on the side of the sphere the unknowns stand on: `q` and `−q` are one
        // attitude, and a hinge holds the child to one sign of its parent's product
        let was: f64 = (0..4).map(|k| q[k] * self.params[a.q[k] as usize].value).sum();
        if was < 0.0 {
            q = q.map(|x| -x);
        }
        for k in 0..4 {
            self.params[a.q[k] as usize].value = q[k];
        }
        self.params[a.d as usize].value = d;
        let at = self.planes[i].att.as_mut().expect("present");
        at.ab = ab;
        at.seat = [q[0], q[1], q[2], q[3], d];
    }

    /// The world length one unit of a view's quaternion is worth: the farthest the view's
    /// content stands from the point it turns about — its origin's in-plane part, its offset,
    /// and every member point — never less than its datum's chord, which is the drawing's own
    /// sense of a size where the view holds nothing yet.  A preconditioner, read once at the
    /// mint: an estimate that drifts costs convergence rate, never correctness.
    fn att_scale(&self, i: usize, ab: [f64; 2], d: f64) -> f64 {
        let f = &self.planes[i].frame;
        let (c, s) = (self.params[f.c as usize].value, self.params[f.s as usize].value);
        let o = self.point_xy(f.origin as usize);
        let far = |a: f64, b: f64| (ab[0] + a).hypot(ab[1] + b).hypot(d);
        let mut r = far(0.0, 0.0);
        for (k, p) in self.points.iter().enumerate() {
            if p.plane == Some(i as u32) {
                let (a, b) = plane::in_view(c, s, o, self.point_xy(k));
                r = r.max(far(a, b));
            }
        }
        r.max(self.frame_chord(f.origin as usize, f.toward as usize).1)
    }

    /// The hidden point in space that is view point `p`'s lift — minted once per point, with
    /// three Params seeded at the lift it has now and the intrinsic row holding it there: `lift`
    /// where the view is solved and `lift_fixed` (the stated basis as constants) where it is not.
    /// `None` for a point on no view: the page lift is P2's, with the role rule that reads a
    /// datum point by what it is related to.
    pub fn lift_point(&mut self, p: usize) -> Option<usize> {
        if let Some(k) = self.lift_of(p) {
            return Some(k);
        }
        let v = self.plane_of(p)?;
        let at = self.world_point(p);
        let name = {
            let x = &self.params[self.points[p].x as usize].name;
            x.strip_suffix(".x").unwrap_or(x).to_string()
        };
        let x = [0, 1, 2].map(|k| {
            self.param(at[k], false, &format!("{name}.lift.{}", ["x", "y", "z"][k])) as u32
        });
        self.lifts.push(LiftE { point: p as u32, x });
        let kind = if self.planes[v].att.is_some() { CKind::Lift } else { CKind::LiftFixed };
        let mut c = Constraint::new(kind, vec![
            crate::constraints::Arg::Ent(EntRef::point(p)),
            crate::constraints::Arg::Ent(EntRef::plane(v)),
        ]);
        c.intrinsic = true;
        self.add(c);
        Some(self.lifts.len() - 1)
    }

    /// Which hidden point lifts view point `p`, if one has been minted.
    pub fn lift_of(&self, p: usize) -> Option<usize> {
        self.lifts.iter().position(|l| l.point as usize == p)
    }

    /// Where view point `p` stands in space as a solve sees it: its hidden point where one has
    /// been minted, and the lift of its drawn pose where not — the same number until a solve has
    /// moved one and not yet the other.
    pub fn lifted(&self, p: usize) -> [f64; 3] {
        match self.lift_of(p) {
            Some(k) => self.lifts[k].x.map(|x| self.params[x as usize].value),
            None => self.world_point(p),
        }
    }
}

/// The unknowns a stated basis is: its quaternion, its origin along the normal, and the rest of
/// its origin in the plane.
fn seated(b: &plane::Basis) -> (Quat, f64, [f64; 2]) {
    let n = b.normal();
    (plane::to_quat(b), plane::dot(b.o, n), [plane::dot(b.o, b.u), plane::dot(b.o, b.v)])
}
