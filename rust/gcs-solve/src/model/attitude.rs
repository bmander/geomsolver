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

    /// Whether direction `e` is held: an axis's outright, a line's by its ends, each held where
    /// it is drawn, in a plane held where it stands.  Anything else is no direction.
    pub fn direction_held(&self, e: EntRef) -> bool {
        match e.kind {
            EntKind::Axis => self.axes[e.i()].d.iter().all(|&p| self.params[p as usize].fixed),
            EntKind::Line => {
                let l = &self.lines[e.i()];
                [l.p1, l.p2].iter().all(|&p| {
                    self.point_held(p as usize)
                        && self.plane_of(p as usize).is_none_or(|v| self.plane_fixed(v))
                })
            }
            _ => false,
        }
    }

    /// Whether every number of point `p` is held: two in its plane, three in space.
    pub fn point_held(&self, p: usize) -> bool {
        let pt = &self.points[p];
        [pt.x, pt.y].into_iter().chain(pt.z).all(|q| self.params[q as usize].fixed)
    }

    /// Frame `f`'s own numbers (#383): where its chart stands, and its joint's ratio and advance,
    /// each where it has one — in the order `frame_members` names them.
    pub fn frame_params(&self, f: usize) -> Vec<u32> {
        let fr = &self.frames[f];
        fr.chart.iter().flat_map(|c| c.o).chain(fr.joint.iter().flat_map(|j| [j.ratio, j.advance]))
            .collect()
    }

    /// The names of `frame_params`, as a `fix` holds them.
    pub fn frame_members(&self, f: usize) -> Vec<&'static str> {
        let fr = &self.frames[f];
        let chart: &[&str] =
            if fr.chart.is_some() { &["origin.x", "origin.y", "origin.z"] } else { &[] };
        let joint: &[&str] = if fr.joint.is_some() { &["ratio", "advance"] } else { &[] };
        chart.iter().chain(joint).copied().collect()
    }

    /// **The jointed frames a path of frames carries** (#383), outermost first: each frame after
    /// the frames it rides on, every one once, the frames with no joint left out — what a thing
    /// drawn in the path moves with.  Empty for the world.
    pub fn carriers_of(&self, path: &[u32]) -> Vec<u32> {
        fn visit(sk: &Sketch, f: u32, depth: usize, out: &mut Vec<u32>) {
            let Some(frame) = sk.frames.get(f as usize) else { return };
            if depth > 64 || out.contains(&f) {
                return;
            }
            for &g in &frame.rides {
                visit(sk, g, depth + 1, out);
            }
            if frame.joint.is_some() && !out.contains(&f) {
                out.push(f);
            }
        }
        let mut out = Vec::new();
        for &f in path {
            visit(self, f, 0, &mut out);
        }
        out
    }

    /// **What `e` rides on** (#383): the jointed frames carrying it, outermost first — a point
    /// its path's (`rides`), a frame its own path's and itself, anything else what its parts ride
    /// on, which must agree: `Err` names two parts that ride on different frames.  A motion, an
    /// axis and a datum ride on nothing.
    pub fn carriers(&self, e: EntRef) -> Result<Vec<u32>, (EntRef, EntRef)> {
        self.carriers_within(e, true, 0).map(|(c, _)| c)
    }

    /// **What `e` rides on as drawn**: its points' paths alone, not the observer a reading gave
    /// what it made — a generating cut rides on its blank, and may still be placed `under:` the
    /// blank's index.
    pub fn drawn_carriers(&self, e: EntRef) -> Result<Vec<u32>, (EntRef, EntRef)> {
        self.carriers_within(e, false, 0).map(|(c, _)| c)
    }

    /// The same, with the part the carriers were found on, for naming a disagreement.
    fn carriers_within(&self, e: EntRef, made: bool, depth: usize)
        -> Result<(Vec<u32>, EntRef), (EntRef, EntRef)> {
        // what is not built yet — a solid naming one after it, refused where it is built —
        // rides on nothing
        if e.i() >= self.count(e.kind) {
            return Ok((Vec::new(), e));
        }
        if let Some(path) = self.rides.get(&e).filter(|_| made || e.kind == EntKind::Point) {
            return Ok((self.carriers_of(path), e));
        }
        match e.kind {
            EntKind::Frame => {
                let mut path = self.frames[e.i()].rides.clone();
                path.push(e.i() as u32);
                Ok((self.carriers_of(&path), e))
            }
            _ if depth > 64 => Ok((Vec::new(), e)),
            // drawn things ride as their points do, and a face or a solid as its edges
            EntKind::Line | EntKind::Circle | EntKind::Arc | EntKind::Spline | EntKind::Face
            | EntKind::Solid | EntKind::Surface => {
                let mut found: Option<(Vec<u32>, EntRef)> = None;
                for k in self.children(e) {
                    let (c, at) = self.carriers_within(k, made, depth + 1)?;
                    match &found {
                        None => found = Some((c, at)),
                        Some((f, _)) if *f == c => {}
                        Some((_, other)) => return Err((*other, at)),
                    }
                }
                Ok(found.unwrap_or((Vec::new(), e)))
            }
            // a point, a datum, a motion, a curve of expressions, or what is evaluated rides
            // where it is said to (`rides`), and otherwise on nothing
            _ => Ok((Vec::new(), e)),
        }
    }

    /// **Mint frame `f`'s joint** (#383): its axis `f.axis`, free and seeded along `z`, and its
    /// two numbers, a ratio of 1 and no advance — what relations then hold.  Once.
    pub fn mint_joint(&mut self, f: usize, name: &str) -> crate::model::Joint {
        if let Some(j) = &self.frames[f].joint {
            return j.clone();
        }
        let axis = self.axis([0.0, 0.0, 1.0], &format!("{name}.axis")) as u32;
        let ratio = self.param(1.0, false, &format!("{name}.ratio")) as u32;
        let advance = self.param(0.0, false, &format!("{name}.advance")) as u32;
        let j = crate::model::Joint { axis, ratio, advance, motion: None };
        self.frames[f].joint = Some(j.clone());
        j
    }

    /// **Every joint's axis seeded where its kind puts it** (#383): along the line it turns about
    /// or slides along and through its start, or square to the view it turns in and through its
    /// centre — after the seeds and the planes are placed, so the solve starts with the axis on
    /// what holds it: a `parallel` started square to its line has no gradient to turn it by.  A
    /// direction or a place a `fix` holds stays.
    pub fn seed_joints(&mut self) {
        for f in 0..self.frames.len() {
            let Some(j) = self.frames[f].joint.clone() else { continue };
            let Some(m) = j.motion else { continue };
            let (at, dir) = match self.motions[m as usize].def {
                MotionDef::Rotation { axis, .. } | MotionDef::Translation { axis, .. } => {
                    let l = &self.lines[axis as usize];
                    let (a, b) = (self.world_point(l.p1 as usize), self.world_point(l.p2 as usize));
                    (a, crate::space::sub(b, a))
                }
                MotionDef::Turn { centre, .. } => {
                    let n = self.plane_of(centre as usize).map(|v| self.basis(v).normal())
                        .unwrap_or([0.0, 0.0, 1.0]);
                    (self.world_point(centre as usize), n)
                }
                // a joint held to an axis no drawn line is — a frame's own, `f turns(r) P.normal`
                // — stands where that axis does, sense and all (#366)
                MotionDef::Joint { axis, .. } => {
                    let Some(o) = self.axis_held_to(axis as usize) else { continue };
                    let v = |q: [u32; 3]| q.map(|i| self.params[i as usize].value);
                    (v(self.axes[o].a), v(self.axes[o].d))
                }
                _ => continue,
            };
            let Some(d) = crate::space::normalised(dir) else { continue };
            let r = j.axis as usize;
            for k in 0..3 {
                let q = self.axes[r].d[k] as usize;
                if !self.params[q].fixed {
                    self.params[q].value = d[k];
                }
            }
            if !self.axes[r].a.iter().any(|&q| self.params[q as usize].fixed) {
                self.stand_axis_through(r, at);
            }
        }
    }

    /// The axis axis `r` is held coincident with (`CKind::AxisCoincident`), the first stated.
    fn axis_held_to(&self, r: usize) -> Option<usize> {
        use crate::constraints::{Arg, CKind};
        self.constraints.iter().filter(|c| c.kind == CKind::AxisCoincident && !c.claim)
            .find_map(|c| match (&c.args[0], &c.args[1]) {
                (Arg::Ent(a), Arg::Ent(b)) if a.i() == r && b.i() != r => Some(b.i()),
                (Arg::Ent(a), Arg::Ent(b)) if b.i() == r && a.i() != r => Some(a.i()),
                _ => None,
            })
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
