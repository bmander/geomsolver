//! Where a plane stands in space, read off its axes and its origin, and the hidden points in
//! space a spatial relation reads (`docs/planes-plan.md`).
//!
//! Nothing here is minted unless asked for: a document that relates no two planes in space has
//! no `LiftE`, so its parameter vector, its constraints and every number a solve reads are what
//! they would be without any of this.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::constraints::CKind;

impl Sketch {
    /// Plane `i` in space: `u` along its first axis, `v` what is left of its second after its
    /// component along `u` is removed — so out of the plane is `u × v` and up is `out × u` —
    /// standing at `o`.  The one reader: every consumer asks here, so a plane whose axes a solve
    /// turns is read the same way as one whose axes a `fix` holds.  Two axes that name no plane
    /// (parallel, or one of no length) read as the front plane's attitude at `o`, which a row
    /// over them will not leave standing.
    pub fn basis(&self, i: usize) -> crate::plane::Basis {
        // a frame with no axes of its own reads as the world does (#383): along x, up y, out z
        let Some(p) = self.frames[i].chart.as_ref() else {
            return crate::plane::Basis { u: [1.0, 0.0, 0.0], v: [0.0, 1.0, 0.0], o: [0.0; 3] };
        };
        let dir = |r: u32| self.axes[r as usize].d.map(|q| self.params[q as usize].value);
        let o = p.o.map(|q| self.params[q as usize].value);
        let page = crate::plane::Basis::page();
        let mut b = crate::plane::Basis::explicit(dir(p.u), dir(p.v)).unwrap_or(page);
        b.o = o;
        b
    }

    /// Whether plane `i` is **fixed**: both its axes' directions and where it stands are held, so
    /// a row reading it may read its basis as constants (`Project`'s fold line, where the
    /// projection's other plane is fixed too: `CKind::attitude_twin`).  What a statement asks at
    /// the add (`Sketch::add_quiet`); a `fix` is applied before any relation.
    pub fn plane_fixed(&self, i: usize) -> bool {
        // a frame with no axes has nothing of its own to move
        let Some(p) = self.frames[i].chart.as_ref() else { return true };
        let held = |q: u32| self.params[q as usize].fixed;
        [p.u, p.v].iter().all(|&r| self.axes[r as usize].d.iter().all(|&q| held(q)))
            && p.o.iter().all(|&q| held(q))
    }

    /// The name point `p`'s numbers are filed under: `a` for `a.x`, `a.y`.
    pub fn point_label(&self, p: usize) -> &str {
        let x = &self.params[self.points[p].x as usize].name;
        x.strip_suffix(".x").unwrap_or(x)
    }

    /// Whether every number of point `p` is held: two in its plane, three in space.
    pub fn point_held(&self, p: usize) -> bool {
        let pt = &self.points[p];
        [pt.x, pt.y].into_iter().chain(pt.z).all(|q| self.params[q as usize].fixed)
    }

    /// The frames with axes of their own — every plane, and each spatial frame whose axes the
    /// document reads (#383): what a walk over the frames' charts walks.
    pub fn charted(&self) -> Vec<usize> {
        (0..self.frames.len()).filter(|&i| self.frames[i].chart.is_some()).collect()
    }

    /// The frames whose `in` draws flat: the planes (#383).
    pub fn planes(&self) -> Vec<usize> {
        (0..self.frames.len()).filter(|&i| self.frames[i].planar()).collect()
    }

    /// The plane point `p` is the origin of, if any: an origin is drawn in its own plane.
    pub fn plane_of_origin(&self, p: usize) -> Option<usize> {
        let pl = self.points[p].plane? as usize;
        (self.frames.get(pl)?.chart.as_ref()?.origin as usize == p).then_some(pl)
    }

    /// Turn axis `r` to drawn line `l` as it now stands, sense and all: the axis a plane holds
    /// along a line (`program::entities::axes_along`), which takes the line's direction only (its
    /// place is the plane's to give).  False where the line has no length.
    pub fn turn_axis_along(&mut self, r: usize, l: usize) -> bool {
        let ln = &self.lines[l];
        let d = crate::space::sub(self.world_point(ln.p2 as usize), self.world_point(ln.p1 as usize));
        let Some(d) = crate::space::normalised(d) else { return false };
        for k in 0..3 {
            let q = self.axes[r].d[k] as usize;
            self.params[q].value = d[k];
        }
        true
    }

    /// The hidden point in space that point `p` stands at, minted once.  A point drawn in a
    /// plane gets three Params seeded where it stands and the intrinsic `lift` row holding them
    /// there, over the plane's axes and origin.  A **point in space** is its own lift: its three
    /// Params, and no row.  `None` for a point of a 2D sketch, which has no place in space
    /// (`program::reading` refuses one).
    pub fn lift_point(&mut self, p: usize) -> Option<usize> {
        if let Some(k) = self.lift_of(p) {
            return Some(k);
        }
        if let Some(z) = self.points[p].z {
            // a point in a spatial frame's chart is lifted through it, as one in a plane is
            if let Some(f) = self.points[p].chart {
                let at = self.world_point(p);
                let name = self.point_label(p).to_string();
                let x = [0, 1, 2].map(|k| {
                    self.param(at[k], false, &format!("{name}.lift.{}", ["x", "y", "z"][k])) as u32
                });
                self.lifts.push(LiftE { point: p as u32, x });
                let mut c = Constraint::new(CKind::ChartLift, vec![
                    crate::constraints::Arg::Ent(EntRef::point(p)),
                    crate::constraints::Arg::Ent(EntRef::frame(f as usize)),
                ]);
                c.intrinsic = true;
                self.add(c);
                return Some(self.lifts.len() - 1);
            }
            let pt = &self.points[p];
            self.lifts.push(LiftE { point: p as u32, x: [pt.x, pt.y, z] });
            return Some(self.lifts.len() - 1);
        }
        let v = self.plane_of(p)?;
        let at = self.world_point(p);
        let name = self.point_label(p).to_string();
        let x = [0, 1, 2].map(|k| {
            self.param(at[k], false, &format!("{name}.lift.{}", ["x", "y", "z"][k])) as u32
        });
        self.lifts.push(LiftE { point: p as u32, x });
        let mut c = Constraint::new(CKind::Lift, vec![
            crate::constraints::Arg::Ent(EntRef::point(p)),
            crate::constraints::Arg::Ent(EntRef::frame(v)),
        ]);
        c.intrinsic = true;
        self.add(c);
        Some(self.lifts.len() - 1)
    }

    /// Whether point `p`'s hidden point in space is a point of its own, held to it by a row: one
    /// drawn in a plane, or in a spatial frame's chart (#383) — not a point in space, its own lift.
    pub fn lifted_off(&self, p: usize) -> bool {
        self.points[p].z.is_none() || self.points[p].chart.is_some()
    }

    /// Whether point `p` has a place in space: drawn in a plane, or standing there itself — not a
    /// point of a 2D sketch, which `lift_point` has nothing to lift.
    pub fn has_place(&self, p: usize) -> bool {
        self.plane_of(p).is_some() || self.points[p].z.is_some()
    }

    /// Which hidden point lifts point `p`, if one has been minted.
    pub fn lift_of(&self, p: usize) -> Option<usize> {
        self.lifts.iter().position(|l| l.point as usize == p)
    }

    /// Where point `p` stands in space as a solve sees it: its hidden point where one has been
    /// minted, and where it stands now where not — the same number until a solve has moved one
    /// and not yet the other.
    pub fn lifted(&self, p: usize) -> [f64; 3] {
        match self.lift_of(p) {
            Some(k) => self.lifts[k].x.map(|x| self.params[x as usize].value),
            None => self.world_point(p),
        }
    }
}
