//! Geometry construction and constraint mutation, preserving parameter and identity order.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::constraints::Arg;

/// The CCW arc through three points.
#[derive(Clone, Copy, Debug)]
pub struct ThreePointArc {
    pub cx: f64,
    pub cy: f64,
    pub r: f64,
    pub a0: f64,
    pub a1: f64,
    /// True when the sweep runs from the *second* given point to the first.
    pub swapped: bool,
}

/// Arc from (ax, ay) to (bx, by) passing through (cx, cy) — the circumcircle of the three, plus
/// the sweep direction that actually contains the third point.  `None` if they are collinear
/// (the test is on the sine of the angle, so it is scale-free).
pub fn three_point_arc(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
    tol: f64,
) -> Option<ThreePointArc> {
    let (ux, uy) = (bx - ax, by - ay);
    let (vx, vy) = (cx - ax, cy - ay);
    let cross = ux * vy - uy * vx;
    if cross.abs() <= tol * ux.dhypot(uy) * vx.dhypot(vy) {
        return None;
    }
    let d = 2.0 * cross;
    let (u2, v2) = (ux * ux + uy * uy, vx * vx + vy * vy);
    let ox = ax + (vy * u2 - uy * v2) / d;
    let oy = ay + (ux * v2 - vx * u2) / d;
    let r = (ax - ox).dhypot(ay - oy);
    let ta = (ay - oy).datan2(ax - ox);
    let tb = (by - oy).datan2(bx - ox);
    let tau = 2.0 * std::f64::consts::PI;
    let sweep = |th: f64| ((th - ta) % tau + tau) % tau;
    let to_b = sweep(tb);
    let to_c = sweep((cy - oy).datan2(cx - ox));
    Some(if to_c < to_b {
        ThreePointArc { cx: ox, cy: oy, r, a0: ta, a1: ta + to_b, swapped: false }
    } else {
        ThreePointArc { cx: ox, cy: oy, r, a0: tb, a1: tb + (tau - to_b), swapped: true }
    })
}

impl Sketch {
    // -- construction -------------------------------------------------------

    pub fn param(&mut self, value: f64, fixed: bool, name: &str) -> usize {
        self.param_scaled(value, fixed, name, 1.0)
    }

    /// A parameter that is not a length: `scale` is the world length one unit of it is worth.
    pub fn param_scaled(&mut self, value: f64, fixed: bool, name: &str, scale: f64) -> usize {
        self.params.push(Param { value, fixed, name: name.to_string(), scale });
        self.params.len() - 1
    }

    pub fn point(&mut self, x: f64, y: f64, fixed: bool, name: &str) -> usize {
        let px = self.param(x, fixed, &format!("{name}.x"));
        let py = self.param(y, fixed, &format!("{name}.y"));
        self.points.push(PointE { x: px as u32, y: py as u32, z: None, plane: None, chart: None });
        self.points.len() - 1
    }

    /// A **point in space**: three coordinates, in no plane.
    pub fn point3(&mut self, at: [f64; 3], fixed: bool, name: &str) -> usize {
        let p = self.point(at[0], at[1], fixed, name);
        self.give_place(p, at[2]);
        p
    }

    /// Make charted frame `f` spatial (#383): a point drawn in it has three numbers, and its
    /// origin, drawn in it, is held at `(0, 0, 0)`.
    pub fn spatial_chart(&mut self, f: usize) {
        self.frames[f].planar = false;
        let origin = self.frames[f].origin as usize;
        self.points[origin].plane = None;
        self.points[origin].chart = Some(f as u32);
        self.give_place(origin, 0.0);
        let z = self.points[origin].z.expect("given its place") as usize;
        self.params[z].fixed = true;
    }

    /// Draw point `p` in spatial frame `f`'s chart (#383): its three numbers, `x` along `f.u`, `y`
    /// up and `z` out, the third given with every point in space's (`give_place`).  A membership
    /// and not a constraint: nothing moves, and its lift reads the chart (`lift_point`).
    pub fn draw_in_chart(&mut self, p: usize, f: usize) {
        debug_assert!(self.frames[f].chart.as_ref().is_some_and(|c| !c.planar));
        self.points[p].chart = Some(f as u32);
    }

    /// Put point `p`, in no plane, in space at height `z`: its third coordinate — what the
    /// elaborator does to a point no `in` reached once every membership is in.
    pub fn give_place(&mut self, p: usize, z: f64) {
        if self.points[p].z.is_some() || self.points[p].plane.is_some() {
            return;
        }
        let fixed = self.params[self.points[p].x as usize].fixed;
        let name = self.point_label(p).to_string();
        let pz = self.param(z, fixed, &format!("{name}.z"));
        self.points[p].z = Some(pz as u32);
    }

    pub fn line(&mut self, p1: usize, p2: usize) -> usize {
        self.lines.push(LineE { p1: p1 as u32, p2: p2 as u32, class: Classes::default() });
        self.lines.len() - 1
    }

    pub fn line_xy(&mut self, x1: f64, y1: f64, x2: f64, y2: f64, name: &str) -> usize {
        let a = self.point(x1, y1, false, &format!("{name}.p1"));
        let b = self.point(x2, y2, false, &format!("{name}.p2"));
        self.line(a, b)
    }

    pub fn circle(&mut self, center: usize, radius: f64, name: &str) -> usize {
        let r = self.param(radius, false, &format!("{name}.r"));
        self.circles.push(CircleE {
            center: center as u32,
            radius: r as u32,
            class: Classes::default(),
        });
        self.circles.len() - 1
    }

    /// An axis: its direction `d` (normalised here, so a seed need not be a unit vector) and the
    /// point on it nearest the origin, `a`, which stays fixed — no freedom of the drawing's —
    /// until a relation reads where the axis is (`place_axis`).  The intrinsic `axis_unit` row is
    /// minted here and nowhere else, since intrinsics are never serialized.
    pub fn axis(&mut self, d: [f64; 3], name: &str) -> usize {
        self.origin_param();
        // normalised only when it is not unit already, so a direction read back is the bits
        // that were written
        let n = crate::space::norm(d);
        let d = if n <= 0.0 {
            [0.0, 0.0, 1.0]
        } else if (n - 1.0).abs() > 1e-12 {
            d.map(|x| x / n)
        } else {
            d
        };
        // a direction is worth the drawing's size: its error moves a far point that much
        let scale = self.extent().max(1.0);
        let dp = ["x", "y", "z"].map(|k| self.param_scaled(0.0, false, &format!("{name}.{k}"), scale) as u32);
        for k in 0..3 {
            self.params[dp[k] as usize].value = d[k];
        }
        let ap = ["px", "py", "pz"].map(|k| self.param(0.0, true, &format!("{name}.{k}")) as u32);
        self.axes.push(AxisE { d: dp, a: ap, placed: false, class: Classes::default() });
        let ri = self.axes.len() - 1;
        let mut c = Constraint::new(crate::constraints::CKind::AxisUnit, vec![crate::constraints::Arg::Ent(EntRef::new(EntKind::Axis, ri))]);
        c.intrinsic = true;
        self.add_quiet(c);
        ri
    }

    /// **A circle's gauge taken again where it stands** (#363 S6): each axis a circle about a
    /// point in space holds its plane's turn by (`round_gauges`) that has come within 45° of the
    /// circle's normal is turned onto the plane's `v` as it now stands, made square to its `u` —
    /// so the row it gauges by still holds, exactly, and the turn is held a quarter turn from
    /// where the row would lose its rank.  A fixed axis can serve no normal it lies along, and
    /// none serves every normal (a field of tangents on the sphere vanishes somewhere), so the
    /// gauge follows the normal instead, taken where each solve ends (`System::solve`); within
    /// one the rows stay consistent where it is passed — `u ⟂ n` then says `u ⟂ e` — so a solve
    /// never fails on it.  Whether any was turned.
    pub fn retake_round_gauges(&mut self) -> bool {
        use crate::space::{cross, dot, norm, sub};
        let mut turned = false;
        for (&g, &[u, v]) in &self.round_gauges.clone() {
            if g.max(u).max(v) as usize >= self.axes.len() {
                continue;
            }
            let read = |a: u32| self.axes[a as usize].d.map(|p| self.params[p as usize].value);
            let (du, dv, dg) = (read(u), read(v), read(g));
            let n = cross(du, dv);
            let (nn, nu, ng) = (norm(n), norm(du), norm(dg));
            if nn <= 0.0 || nu <= 0.0 || ng <= 0.0
                || dot(n, dg).abs() <= std::f64::consts::FRAC_1_SQRT_2 * nn * ng
            {
                continue;
            }
            let uh = du.map(|x| x / nu);
            let e = sub(dv, uh.map(|x| x * dot(dv, uh)));
            let ne = norm(e);
            if ne <= 0.0 {
                continue;
            }
            for (k, p) in self.axes[g as usize].d.into_iter().enumerate() {
                self.params[p as usize].value = e[k] / ne;
            }
            turned = true;
        }
        turned
    }

    /// Free axis `i`'s place and hold it to the foot of the perpendicular from the origin (the
    /// intrinsic `axis_foot` row) — once a relation reads where the axis is, and not before: an axis
    /// read only as a direction has a place no equation mentions, which would be counted as two
    /// freedoms of a drawing that has none.
    pub fn place_axis(&mut self, i: usize) {
        if self.axes[i].placed {
            return;
        }
        self.axes[i].placed = true;
        for p in self.axes[i].a {
            self.params[p as usize].fixed = false;
        }
        let mut c = Constraint::new(crate::constraints::CKind::AxisFoot, vec![crate::constraints::Arg::Ent(EntRef::new(EntKind::Axis, i))]);
        c.intrinsic = true;
        self.add_quiet(c);
    }

    /// `place_axis` undone: the place held where it stands, and the foot row gone with it.
    fn unplace_axis(&mut self, i: usize) {
        if !self.axes[i].placed {
            return;
        }
        self.axes[i].placed = false;
        for p in self.axes[i].a {
            self.params[p as usize].fixed = true;
        }
        let axis = EntRef::new(EntKind::Axis, i);
        self.constraints.retain(|c| {
            !(c.intrinsic && c.kind == crate::constraints::CKind::AxisFoot && c.args[0].ent() == axis)
        });
    }

    /// A fixed Param holding 0, shared by every axis: the kernels that read a line's direction as
    /// `B − A` read an axis's as the segment `(0, d)`, so an axis needs no kernels of its own for
    /// `parallel`, `perpendicular` and `angle`.
    pub fn origin_param(&mut self) -> u32 {
        if let Some(z) = self.zero {
            return z;
        }
        let z = self.param(0.0, true, "zero") as u32;
        self.zero = Some(z);
        z
    }

    /// An arc plus its two intrinsic `PointOnCircle` constraints.
    pub fn arc(&mut self, center: usize, start: usize, end: usize, name: &str) -> usize {
        let (cx, cy) = self.point_xy(center);
        let (sx, sy) = self.point_xy(start);
        let r = (sx - cx).dhypot(sy - cy);
        let rp = self.param(r, false, &format!("{name}.r"));
        self.arcs.push(ArcE {
            center: center as u32,
            start: start as u32,
            end: end as u32,
            radius: rp as u32,
            class: Classes::default(),
        });
        let ai = self.arcs.len() - 1;
        let aref = EntRef::arc(ai);
        let c1 = Constraint::point_on_circle(EntRef::point(start), aref, true);
        let c2 = Constraint::point_on_circle(EntRef::point(end), aref, true);
        self.add(c1);
        self.add(c2);
        ai
    }

    /// Arc from `start` to `end` bulging through `through` — the three-point construction.
    /// Creates the centre point; `None` if the three are collinear.
    pub fn arc_through(
        &mut self,
        start: usize,
        end: usize,
        through: (f64, f64),
        name: &str,
    ) -> Option<usize> {
        let (ax, ay) = self.point_xy(start);
        let (bx, by) = self.point_xy(end);
        let g = three_point_arc(ax, ay, bx, by, through.0, through.1, 1e-9)?;
        let centre = self.point(g.cx, g.cy, false, &format!("{name}.c"));
        let (a, b) = if g.swapped { (end, start) } else { (start, end) };
        Some(self.arc(centre, a, b, name))
    }

    /// A plane over two axes — right along `u`'s, out along `u × v` — standing at `o`, three
    /// Params of its own, and its origin, a point drawn in it and held at `(0, 0)`, minted here.
    /// No intrinsic row: what the plane is, `Sketch::basis` reads off the axes and `o`.
    pub fn plane(&mut self, u: usize, v: usize, o: [f64; 3], name: &str) -> usize {
        let op = self.plane_place(o, name);
        let origin = self.point(0.0, 0.0, true, &format!("{name}.origin"));
        self.push_plane(u, v, op, origin)
    }

    /// A plane whose origin is a point already made — what a document reader and the rebuild
    /// walk build, the point having come with the others.  It is put in the plane and held at
    /// `(0, 0)`.
    pub fn plane_over(&mut self, u: usize, v: usize, o: [f64; 3], origin: usize, name: &str) -> usize {
        let op = self.plane_place(o, name);
        for p in self.point_params(origin) {
            self.params[p as usize].value = 0.0;
            self.params[p as usize].fixed = true;
        }
        self.push_plane(u, v, op, origin)
    }

    /// An axis as a document or another sketch carries it: its direction and which of the three
    /// are held.  Where it stands is put back by `place_restored`, once the planes over it are.
    pub fn axis_restored(&mut self, d: [f64; 3], held: [bool; 3], class: Classes) -> usize {
        let ri = self.axis(d, "");
        for k in 0..3 {
            let dp = self.axes[ri].d[k] as usize;
            self.params[dp].fixed = held[k];
        }
        self.axes[ri].class = class;
        ri
    }

    /// Where axis `r` stands, as a document or another sketch carries it, put back after the
    /// planes over it are built (a plane stands an axis it is first to place through its own
    /// origin): its numbers, and where any is held, placed and those held — the order an
    /// elaboration's gauge pass, after the planes, comes to.  Read by a relation and held by
    /// none, it is placed when the relation is added.
    pub fn place_restored(&mut self, r: usize, a: [f64; 3], held: [bool; 3]) {
        for k in 0..3 {
            let q = self.axes[r].a[k] as usize;
            self.params[q].value = a[k];
        }
        self.hold_place(r, held);
    }

    /// Hold those of axis `r`'s place numbers `held` says, placing it first where any is, so no
    /// relation added later frees what is held (`place_axis`).  A place nothing holds and
    /// nothing reads is left as it is: held, and no freedom.
    pub fn hold_place(&mut self, r: usize, held: [bool; 3]) {
        if !held.contains(&true) {
            return;
        }
        self.place_axis(r);
        for k in 0..3 {
            let q = self.axes[r].a[k] as usize;
            self.params[q].fixed = held[k];
        }
    }

    /// A plane as a document or another sketch carries it: over axes `u`, `v`, standing at `o`
    /// with which of its three numbers are held, its origin a point already made.
    pub fn plane_restored(&mut self, u: usize, v: usize, o: [f64; 3], held: [bool; 3], origin: usize,
                          class: Classes) -> usize {
        let pi = self.plane_over(u, v, o, origin, "");
        for k in 0..3 {
            let q = self.frames[pi].o[k] as usize;
            self.params[q].fixed = held[k];
        }
        self.frames[pi].class = class;
        pi
    }

    /// The three Params of where a plane stands, `name.x` … `name.z`.
    fn plane_place(&mut self, o: [f64; 3], name: &str) -> [u32; 3] {
        [0, 1, 2].map(|k| self.param(o[k], false, &format!("{name}.{}", ["x", "y", "z"][k])) as u32)
    }

    /// A plane over axes `u`, `v`, standing at `op`, its origin drawn in it.
    /// Its axes pass through its origin: the intrinsic `PlaneAxis` rows, which read (and so place)
    /// both axes.
    fn push_plane(&mut self, u: usize, v: usize, op: [u32; 3], origin: usize) -> usize {
        let pi = self.frames.len();
        self.points[origin].plane = Some(pi as u32);
        self.frames.push(FrameE::charted(Chart { u: u as u32, v: v as u32, o: op, origin: origin as u32,
                                                 normal: None, planar: true }));
        let o = op.map(|q| self.params[q as usize].value);
        for ax in [u, v] {
            // an axis this plane is the first to place starts through where the plane does
            if !self.axes[ax].placed {
                self.stand_axis_through(ax, o);
            }
            let mut c = Constraint::new(crate::constraints::CKind::PlaneAxis,
                vec![Arg::Ent(EntRef::frame(pi)), Arg::Ent(EntRef::new(EntKind::Axis, ax))]);
            c.intrinsic = true;
            self.add_quiet(c);
        }
        pi
    }

    /// **Plane `pi`'s normal, `P.normal`**: an axis along `u × v` through its origin, minted the
    /// first time it is asked for, under `name`.  Held so by three intrinsic rows — square to
    /// each axis (`Perpendicular3`) and through the origin (`PlaneAxis`) — which leave it its
    /// sense, so it is seeded along `u × v` (`seed_normals`) and a solve, which moves it
    /// continuously, keeps it there.  Three unknowns and three rows for its direction, the unit
    /// row among them, and its place's two against `PlaneAxis`'s two: no freedom of its own.
    pub fn plane_normal(&mut self, pi: usize, name: &str) -> usize {
        if let Some(n) = self.frames[pi].normal {
            return n as usize;
        }
        let n = self.axis(self.basis(pi).normal(), name);
        self.stand_normal(pi, n);
        n
    }

    /// Axis `n` made plane `pi`'s normal: its rows, and it seeded so (`plane_normal`).  What a
    /// document reader calls with an axis already made.
    pub fn stand_normal(&mut self, pi: usize, n: usize) {
        use crate::constraints::{Arg, CKind};
        self.frames[pi].normal = Some(n as u32);
        let (u, v) = (self.frames[pi].u as usize, self.frames[pi].v as usize);
        let axis = EntRef::new(EntKind::Axis, n);
        for side in [u, v] {
            let mut c = Constraint::two_line(CKind::Perpendicular3, axis, EntRef::new(EntKind::Axis, side));
            c.intrinsic = true;
            self.add_quiet(c);
        }
        let mut c = Constraint::new(CKind::PlaneAxis, vec![Arg::Ent(EntRef::frame(pi)), Arg::Ent(axis)]);
        c.intrinsic = true;
        self.add_quiet(c);
        self.seed_normal(pi);
    }

    /// Every plane's normal turned along `u × v` as its axes now stand, and stood through where
    /// it stands — after anything that moves a plane's seed, since the normal's rows hold it
    /// square to the axes either way round.
    pub fn seed_normals(&mut self) {
        for pi in 0..self.frames.len() {
            self.seed_normal(pi);
        }
    }

    fn seed_normal(&mut self, pi: usize) {
        let Some(n) = self.frames[pi].chart.as_ref().and_then(|c| c.normal).map(|n| n as usize) else {
            return;
        };
        let b = self.basis(pi);
        for (k, x) in b.normal().into_iter().enumerate() {
            let q = self.axes[n].d[k] as usize;
            if !self.params[q].fixed {
                self.params[q].value = x;
            }
        }
        // a place a `fix` holds stays where it is held
        let held = self.axes[n].placed && self.axes[n].a.iter().any(|&q| self.params[q as usize].fixed);
        if !held {
            self.stand_axis_through(n, b.o);
        }
    }

    /// **Every point a motion carries to a stated roll seeded where the motion carries it**
    /// (#366, a `CarriedPoint` at a stated roll): in space at `Pose(τ)·x`, drawn in a plane at its
    /// shadow there.
    /// The rows are linear in the point, so the solve would find it from anywhere; but a plane
    /// drawn over carried points is turned along its lines before the solve (`views::align`), and
    /// a frame carried past a quarter turn would stand its axes against its lines from where it
    /// was seeded.  Whether any point moved; the caller puts the hidden points back and places
    /// the planes again.
    pub fn seed_carried(&mut self) -> bool {
        use crate::expr::{self, Aff};
        let mut moved = false;
        // a roll written over a face family's running number is worked out at the station the
        // sketch holds, before the document's expressions are (`expr::evaluate`), as they will be
        self.refresh_held();
        let held: BTreeMap<String, Aff> = self.held.iter()
            .map(|(n, h)| (n.clone(), Aff::of_dim(self.params[h.param as usize].value, h.dim)))
            .collect();
        let stated = |a: &Arg| -> f64 {
            let Arg::Expr(e) = a else { return a.num() };
            expr::parse_in(&e.text, self.units).ok()
                .and_then(|p| expr::eval(&p.body, &held).ok())
                .and_then(|v| v.number())
                .map_or(e.value, |v| expr::to_arg_units(crate::constraints::SpecKind::Angle, v))
        };
        for ci in 0..self.constraints.len() {
            let c = &self.constraints[ci];
            if !c.kind.reads_motion() || !c.stated_roll() || !c.states_rows() {
                continue;
            }
            let (Arg::Ent(x), Arg::Ent(of), Arg::Ent(m)) = (&c.args[0], &c.args[1], &c.args[2])
            else { continue };
            let roll = match c.free {
                Some(f) => f.m * self.params[f.param as usize].value + f.c,
                None => stated(&c.args[3]),
            };
            let Ok(family) = crate::motion::Family::read(self, m.i()) else { continue };
            let Ok(pose) = family.at(roll) else { continue };
            let to = pose.point(self.world_point(of.i()));
            let p = &self.points[x.i()];
            let seeds = match (p.z, p.plane) {
                (Some(z), _) => vec![(p.x, to[0]), (p.y, to[1]), (z, to[2])],
                (None, Some(pl)) => {
                    let (a, b) = self.basis(pl as usize).view_coords(to);
                    vec![(p.x, a), (p.y, b)]
                }
                _ => continue,
            };
            for (q, v) in seeds {
                let q = &mut self.params[q as usize];
                if !q.fixed && v.is_finite() && (q.value - v).abs() > 1e-12 * (1.0 + v.abs()) {
                    q.value = v;
                    moved = true;
                }
            }
        }
        moved
    }

    /// The plane axis `r` is the normal of, where it is one.
    pub fn normal_of(&self, r: usize) -> Option<usize> {
        self.frames.iter().position(|p| p.chart.as_ref().is_some_and(|c| c.normal == Some(r as u32)))
    }

    /// Stand axis `r`'s line through `x`, its direction kept: its place is the foot of the
    /// perpendicular from the world origin to that line.
    pub fn stand_axis_through(&mut self, r: usize, x: [f64; 3]) {
        let d = self.axes[r].d.map(|p| self.params[p as usize].value);
        let Some(d) = crate::space::normalised(d) else { return };
        let foot = crate::space::sub(x, crate::space::scale(d, crate::space::dot(x, d)));
        for k in 0..3 {
            let q = self.axes[r].a[k] as usize;
            self.params[q].value = foot[k];
        }
    }

    /// A plane fixed where `b` stands — its axes and its origin held — for a caller that has a
    /// basis in hand rather than a document: a test, a drawing's measuring plane.
    pub fn fixed_plane(&mut self, b: crate::plane::Basis, name: &str) -> usize {
        let axes = [(b.u, "u"), (b.v, "v")].map(|(d, k)| {
            let r = self.axis(d, &format!("{name}.{k}"));
            for &p in &self.axes[r].d {
                self.params[p as usize].fixed = true;
            }
            r
        });
        let pi = self.plane(axes[0], axes[1], b.o, name);
        for &p in &self.frames[pi].o {
            self.params[p as usize].fixed = true;
        }
        // its axes, stood through where it stands, held there
        for r in axes {
            self.hold_place(r, [true; 3]);
        }
        pi
    }

    /// Stand plane `i` at `o`, its axes turned no way and moved with it, so they still pass
    /// through its origin — for a caller that holds a sketch and moves a part rigidly, never for
    /// a solve.
    pub fn set_plane_origin(&mut self, i: usize, o: [f64; 3]) {
        for k in 0..3 {
            let p = self.frames[i].o[k] as usize;
            self.params[p].value = o[k];
        }
        let (u, v) = (self.frames[i].u as usize, self.frames[i].v as usize);
        self.stand_axis_through(u, o);
        self.stand_axis_through(v, o);
    }

    /// A cubic B-spline over `ctrl`, with the clamped uniform knot vector.  `None` if there are
    /// too few control points for a cubic — the curve would have no span to live on.
    pub fn spline(&mut self, ctrl: &[usize]) -> Option<usize> {
        self.spline_with(ctrl, None)
    }

    /// A cubic B-spline with a knot vector of its own — a repeated interior knot is a corner.
    /// `None` if the knots are not ones this control polygon can be drawn with.
    pub fn spline_with(&mut self, ctrl: &[usize], knots: Option<Vec<f64>>) -> Option<usize> {
        self.spline_weighted(ctrl, knots, None)
    }

    /// A rational cubic B-spline: a weight per control point (`curve::weigh`), `None` or all 1
    /// being the polynomial curve, which is stored without them.  `None` where the knots or the
    /// weights are not ones this control polygon can be drawn with.
    pub fn spline_weighted(
        &mut self,
        ctrl: &[usize],
        knots: Option<Vec<f64>>,
        weights: Option<Vec<f64>>,
    ) -> Option<usize> {
        if ctrl.iter().any(|&c| c >= self.points.len()) {
            return None;
        }
        let knots = knots.unwrap_or_else(|| crate::curve::clamped_uniform(ctrl.len()));
        if !crate::curve::knots_valid(&knots, ctrl.len()) {
            return None;
        }
        if weights.as_ref().is_some_and(|w| !crate::curve::weights_valid(w, ctrl.len())) {
            return None;
        }
        self.splines.push(SplineE {
            ctrl: ctrl.iter().map(|&c| c as u32).collect(),
            knots,
            weights: weights.filter(|w| w.iter().any(|&x| x != 1.0)),
            class: Classes::default(),
        });
        Some(self.splines.len() - 1)
    }

    /// A cubic B-spline that passes through `pts`, in order.  The control points are computed,
    /// not clicked — the same bargain `arc_through` strikes: the third click of a three-point
    /// arc is construction input, not a sketch point.  `None` if there are too few points for a
    /// cubic, or they give no parameterisation.
    pub fn spline_through(&mut self, pts: &[(f64, f64)]) -> Option<usize> {
        self.spline_through_held(pts, &[])
    }

    /// The same, holding the curve to the places that came from a Point rather than from empty
    /// space: each becomes a `PointOnSpline` whose parameter is *pinned* at the value the fit
    /// chose for it.
    ///
    /// The pin is what makes the answer determinate.  A contact whose parameter is free says
    /// only "the curve passes through here somewhere along its length", so a curve through m
    /// points keeps m degrees of freedom — it can slide along itself and still meet every one of
    /// them.  The fit already worked out where along, so that is knowledge and not an unknown,
    /// and a curve fitted to fully constrained points comes out fully constrained.
    pub fn spline_through_held(
        &mut self,
        pts: &[(f64, f64)],
        hold: &[Option<usize>],
    ) -> Option<usize> {
        // a short `hold` holds nothing further: the two lists are one-to-one as far as it goes
        if hold.len() > pts.len() || hold.iter().flatten().any(|&p| p >= self.points.len()) {
            return None;
        }
        let (ctrl, knots, at) = crate::curve::interpolating_ctrl(pts)?;
        let ids: Vec<usize> = ctrl
            .iter()
            .enumerate()
            .map(|(i, &(x, y))| self.point(x, y, false, &format!("k{i}")))
            .collect();
        let s = self.spline_with(&ids, Some(knots))?;
        for (i, held) in hold.iter().enumerate() {
            let Some(p) = *held else { continue };
            // pinned: the fit worked out where along the curve this point sits, so that is
            // knowledge and not something to solve for
            let c = Constraint::new(
                crate::constraints::CKind::PointOnSpline,
                vec![
                    Arg::Ent(EntRef::point(p)),
                    Arg::Ent(EntRef::spline(s)),
                    Arg::Seed { value: at[i], pinned: true },
                ],
            );
            self.add(c);
        }
        Some(s)
    }

    /// Four lines round the corners `a` and (x1, y1), sharing corner points, with three
    /// perpendicular constraints.  Three, not four: the fourth follows, so adding it would make
    /// every rectangle over-constrained by one equation.  What is left is the 5 DOF a rectangle
    /// has — position, rotation, width, height.
    pub fn rectangle(&mut self, a: usize, x1: f64, y1: f64, name: &str) -> Vec<usize> {
        let (x0, y0) = self.point_xy(a);
        let corners = [
            a,
            self.point(x1, y0, false, &format!("{name}.b")),
            self.point(x1, y1, false, &format!("{name}.c")),
            self.point(x0, y1, false, &format!("{name}.d")),
        ];
        let lines: Vec<usize> =
            (0..4).map(|i| self.line(corners[i], corners[(i + 1) % 4])).collect();
        for i in 0..3 {
            let c = Constraint::two_line(
                crate::constraints::CKind::Perpendicular,
                EntRef::line(lines[i]),
                EntRef::line(lines[i + 1]),
            );
            self.add(c);
        }
        lines
    }

    pub fn rectangle_xy(&mut self, x0: f64, y0: f64, x1: f64, y1: f64, name: &str) -> Vec<usize> {
        let a = self.point(x0, y0, false, &format!("{name}.a"));
        self.rectangle(a, x1, y1, name)
    }

    /// Append a constraint, assigning it a fresh document-stable id.
    ///
    /// This is also where a constraint's own hidden unknowns become real Params: a `Param` slot
    /// arrives holding the seed number (from `constraints::seed_param`, from a document, or from
    /// the caller) and leaves holding the index of the Param that now carries it.  Doing it here
    /// and only here means a constraint is a number on the way in — which is what a document
    /// stores and what `graft` copies — and an index everywhere the solver looks at it.
    pub fn add(&mut self, c: Constraint) -> u32 {
        let expr = crate::expr::has_expr(&c.args);
        let id = self.add_quiet(c);
        if expr {
            crate::expr::evaluate(self);   // its text may read names, or define one others read
        }
        // an energy, or a row an energy's curve may be held by (#121)
        self.settle_variational();
        id
    }

    /// `add` without the expression pass, for a caller adding a whole document one constraint at
    /// a time and evaluating once at the end — `graft`, the rebuild walk behind deletion,
    /// copying, pasting and the part a drag works on.  Evaluating per add would parse every
    /// expression in the document again for each one that carries text, and would make a
    /// dimension whose definition has not been grafted yet briefly a free variable — allocating
    /// an unknown the next pass immediately retires.
    pub fn add_quiet(&mut self, mut c: Constraint) -> u32 {
        // the twin its planes can feed, decided before anything is minted for it: a projection
        // where either plane moves in the solve reads both images' lifts and both planes' axes,
        // and over two fixed planes only the two drawn points (`CKind::attitude_twin`)
        let reads = c.attitudes_read(self);
        if !reads.is_empty() {
            let solved = reads.iter().any(|&v| !self.plane_fixed(v));
            c.kind = c.kind.attitude_twin(solved);
        }
        // an ordinate's form is its operands' (`constraints::ordinate_form`), read here and only
        // here, so whatever a caller handed in — a document, a paste, a binding's record — the
        // kernel is the one the points and the direction give it now
        if let Some(i) = c.kind.form_slot() {
            if let (Arg::Ent(p), Arg::Ent(q), Arg::Ent(t)) = (&c.args[0], &c.args[1], &c.args[2]) {
                let word = crate::constraints::ordinate_word(c.kind, &c.args);
                let f = crate::constraints::ordinate_form(self, p.i(), q.i(), *t, word);
                c.args[i] = Arg::Int(f as i64);
            }
        }
        if c.id == 0 {
            self.next_cid += 1;
            c.id = self.next_cid;
        } else {
            self.next_cid = self.next_cid.max(c.id);
        }
        let id = c.id;
        // a relation in space reads the hidden points its drawn operands lift to, so they are
        // minted here — after the id, so a constraint that arrived carrying one keeps it — and
        // the statement takes the twin its plane can feed; a point on no view has no lift, and
        // `constraints::validate` is where that is refused
        for p in c.lifted_points(self) {
            self.lift_point(p);
        }
        // and a derivative reads its line's ends there too, and a tangent column for every
        // column its use moves by one (§6.21)
        if let Some(d) = c.along {
            if let super::Toward::Line(l) = self.duals[d].toward {
                for p in self.line_ends(l) {
                    self.lift_point(p);
                }
            }
            let cols = c.row_params(self);
            self.mint_tangents(d, &cols);
        }
        // a relation that reads where an axis is gives the axis its place
        for r in c.axes_placed_by() {
            if r < self.axes.len() {
                self.place_axis(r);
            }
        }
        // **the tangent at a point reads the place of the contact holding that point** (#251): one
        // unknown, the contact's, whatever this statement arrived carrying — a seed, a pin, a
        // name — so a copy or a document read back shares it again
        if let Some((slots, at, on)) = c.tangent_at() {
            if let Some(place) = self.holding(at, on) {
                for (&t, p) in slots.iter().zip(place) {
                    c.args[t] = Arg::Param(p);
                }
            }
        }
        for (i, name) in c.kind.param_slots() {
            if matches!(c.args[i], Arg::Param(_)) {
                continue;   // already allocated (a constraint moved between sketches)
            }
            // a carried point's roll the statement states is no unknown of its own (#366)
            if c.kind.roll_slot() == Some(i) && matches!(c.args[i], Arg::Expr(_)) {
                continue;
            }
            // a contact pinned to a face family's running number owns its parameter, held
            let held = match &c.args[i] {
                Arg::Shared { name, .. } => self.held.get(name).copied(),
                _ => None,
            };
            if let Some(h) = held {
                c.args[i] = Arg::Param(h.param);
                continue;
            }
            if let Arg::Shared { name, seed } = &c.args[i] {
                // a shared unknown is allocated once, by the first contact naming it; a later one
                // owns the same index, and one added after the last owner was removed revives it
                // from its own seed, as a retired free variable is (`expr::free_param`)
                let p = match self.shared.get(name) {
                    Some(place) => {
                        let p = &mut self.params[place.param as usize];
                        if let (true, Some(v)) = (p.fixed, seed) {
                            p.value = *v;
                        }
                        p.fixed = false;
                        place.param
                    }
                    None => {
                        let v = seed.unwrap_or_else(|| {
                            crate::constraints::seed_param(self, c.kind, &c.args, i)
                        });
                        let scale = crate::constraints::param_scale(self, c.kind, &c.args, i);
                        let p = self.param_scaled(v, false, &format!("~{name}"), scale) as u32;
                        // `validate` has already refused a contact that names no curve
                        if let Some(along) = crate::constraints::contact_carrier(c.kind, &c.args) {
                            let place = super::SharedPlace { param: p, along };
                            self.shared.insert(name.clone(), place);
                        }
                        p
                    }
                };
                c.args[i] = Arg::Param(p);
                continue;
            }
            let (v, pinned) = match c.args[i] {
                Arg::Seed { value, pinned } => (value, pinned),
                ref a => (a.num(), false),
            };
            let scale = crate::constraints::param_scale(self, c.kind, &c.args, i);
            let p = self.param_scaled(v, pinned, &format!("c{id}.{name}"), scale);
            c.args[i] = Arg::Param(p as u32);
        }
        // and a contact arriving after a tangent at its point gives that tangent its place
        let holds = c.holds_on();
        self.constraints.push(c);
        if let Some((at, on, place)) = holds {
            self.share_place(at, on, place);
        }
        // the private geometry's own row, moving as the use does
        self.twin_new_intrinsic(self.constraints.len() - 1);
        id
    }

    /// The place the contact holding point `at` on curve (or envelope) `on` owns: the first such
    /// contact's unknowns, where one is in the sketch.
    pub fn holding(&self, at: usize, on: EntRef) -> Option<Vec<u32>> {
        self.constraints.iter().find_map(|c| match c.holds_on() {
            Some((a, e, p)) if a == at && e == on => Some(p),
            _ => None,
        })
    }

    /// Every tangent at point `at` of curve `on` reads place `place` (#251): one stated before
    /// the contact holding its point took unknowns of its own, retired here once nothing owns
    /// them.
    fn share_place(&mut self, at: usize, on: EntRef, place: Vec<u32>) {
        // a second contact holding the point there leaves the tangent with the first's
        if self.holding(at, on).as_ref() != Some(&place) {
            return;
        }
        let mut retired = Vec::new();
        for c in self.constraints.iter_mut() {
            let Some((slots, a, e)) = c.tangent_at() else { continue };
            if a != at || e != on {
                continue;
            }
            for (&t, &p) in slots.iter().zip(&place) {
                if let Arg::Param(q) = c.args[t] {
                    if q != p {
                        retired.push(q);
                    }
                }
                c.args[t] = Arg::Param(p);
            }
        }
        for q in retired {
            if !self.constraints.iter().any(|c| c.owns(q)) {
                self.params[q as usize].fixed = true;
            }
        }
    }

    /// Drop a constraint.  Its own unknowns stay in the parameter vector — every index above
    /// them names something — but are retired to `fixed`, since a free parameter no equation
    /// mentions is a degree of freedom the sketch does not actually have, and diagnosis would
    /// report it.  The rebuild walk (`graft::without`) is the path that reclaims the slots.
    pub fn remove(&mut self, id: u32) {
        let (expr, own) = self.constraint(id)
            .map_or((false, Vec::new()), |c| (crate::expr::has_expr(&c.args), c.aux_params()));
        let placed: Vec<usize> =
            self.constraint(id).map(|c| c.axes_placed_by().collect()).unwrap_or_default();
        let dual = self.constraint(id).and_then(|c| c.along);
        self.constraints.retain(|c| c.id != id);
        // an axis whose place nothing reads now is held again: its place is no freedom
        for r in placed {
            if !self.constraints.iter().any(|c| c.axes_placed_by().any(|s| s == r)) {
                self.unplace_axis(r);
            }
        }
        // a shared unknown stays free while another contact still owns it — one shared by name
        // (`t == s`) or a contact's place read by a tangent at its point (#251)
        for p in own {
            if !self.constraints.iter().any(|c| c.owns(p)) {
                self.params[p as usize].fixed = true;
            }
        }
        if let Some(d) = dual {
            self.retire_tangents(d);
        }
        self.placements.remove(&id);
        if expr {
            // it may have defined a name others read, or been the last reader of a free one
            crate::expr::evaluate(self);
        }
        self.settle_variational();
    }

    pub fn constraint(&self, id: u32) -> Option<&Constraint> {
        self.constraints.iter().find(|c| c.id == id)
    }

    /// Set a numeric argument on one constraint — a dimension, a flag, a count — and bring the
    /// document's expressions back into step when the write replaced one.  Whoever sets a number
    /// means the number, so the expression goes; but it may have defined a name others read, or
    /// have been the last reader of a free variable, and neither can be left as it was.
    ///
    /// `false` when there is no such constraint or no such argument, exactly as `set_num`.
    pub fn set_constraint_num(&mut self, id: u32, name: &str, v: f64) -> bool {
        let Some(c) = self.constraint_mut(id) else { return false };
        let was = c.arg_index(name).is_some_and(|i| matches!(c.args[i], Arg::Expr(_)));
        if !c.set_num(name, v) {
            return false;
        }
        if was {
            crate::expr::evaluate(self);
        }
        true
    }

    pub fn constraint_mut(&mut self, id: u32) -> Option<&mut Constraint> {
        self.constraints.iter_mut().find(|c| c.id == id)
    }

    pub fn index_of(&self, id: u32) -> Option<usize> {
        self.constraints.iter().position(|c| c.id == id)
    }

    pub fn n_residuals(&self) -> usize {
        self.constraints.iter().filter(|c| c.states_rows()).map(|c| c.rows_in(self)).sum()
    }

    /// Constraints the user added (excludes intrinsic and soft/transient ones).
    pub fn user_constraints(&self) -> Vec<&Constraint> {
        self.constraints.iter().filter(|c| !(c.intrinsic || c.soft)).collect()
    }

    /// Everything that must be satisfied: excludes soft ones such as drag targets, and claims,
    /// which are judged rather than satisfied.  This is the named half of the rule the solve
    /// seams spell out inline — a caller that only wants the list asks here, so a consumer added
    /// later inherits both exclusions instead of having to remember them.
    pub fn hard_constraints(&self) -> Vec<&Constraint> {
        self.constraints.iter().filter(|c| c.acts()).collect()
    }

    pub fn hard_ids(&self) -> Vec<u32> {
        self.constraints.iter().filter(|c| c.acts()).map(|c| c.id).collect()
    }
}
