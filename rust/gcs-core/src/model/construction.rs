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
        self.points.push(PointE { x: px as u32, y: py as u32, z: None, plane: None });
        self.points.len() - 1
    }

    /// A **point in space**: three coordinates, in no plane.
    pub fn point3(&mut self, at: [f64; 3], fixed: bool, name: &str) -> usize {
        let p = self.point(at[0], at[1], fixed, name);
        self.give_place(p, at[2]);
        p
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

    /// A sphere about a drawn point, its radius a Param like a circle's.
    pub fn sphere(&mut self, center: usize, radius: f64, name: &str) -> usize {
        let r = self.param(radius, false, &format!("{name}.r"));
        self.spheres.push(SphereE {
            center: center as u32,
            radius: r as u32,
            class: Classes::default(),
        });
        self.spheres.len() - 1
    }

    /// A cone about a drawn line — its apex the line's start — its half-angle a Param (radians).
    pub fn cone(&mut self, axis: usize, half: f64, name: &str) -> usize {
        let a = self.param(half, false, &format!("{name}.half"));
        let class = Classes::default();
        self.cones.push(AxialE { axis: axis as u32, param: a as u32, class });
        self.cones.len() - 1
    }

    /// A cylinder about a drawn line, its radius a Param like a sphere's.
    pub fn cylinder(&mut self, axis: usize, radius: f64, name: &str) -> usize {
        let r = self.param(radius, false, &format!("{name}.r"));
        let class = Classes::default();
        self.cylinders.push(AxialE { axis: axis as u32, param: r as u32, class });
        self.cylinders.len() - 1
    }

    /// A ray: its direction `d` (normalised here, so a seed need not be a unit vector) and the
    /// point on it nearest the origin, `a`, which stays fixed — no freedom of the drawing's —
    /// until a relation reads where the ray is (`place_ray`).  The intrinsic `ray_unit` row is
    /// minted here and nowhere else, since intrinsics are never serialized.
    pub fn ray(&mut self, d: [f64; 3], name: &str) -> usize {
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
        self.rays.push(RayE { d: dp, a: ap, placed: false, class: Classes::default() });
        let ri = self.rays.len() - 1;
        let mut c = Constraint::new(crate::constraints::CKind::RayUnit, vec![crate::constraints::Arg::Ent(EntRef::new(EntKind::Ray, ri))]);
        c.intrinsic = true;
        self.add_quiet(c);
        ri
    }

    /// Free ray `i`'s place and hold it to the foot of the perpendicular from the origin (the
    /// intrinsic `ray_foot` row) — once a relation reads where the ray is, and not before: a ray
    /// read only as a direction has a place no equation mentions, which would be counted as two
    /// freedoms of a drawing that has none.
    pub fn place_ray(&mut self, i: usize) {
        if self.rays[i].placed {
            return;
        }
        self.rays[i].placed = true;
        for p in self.rays[i].a {
            self.params[p as usize].fixed = false;
        }
        let mut c = Constraint::new(crate::constraints::CKind::RayFoot, vec![crate::constraints::Arg::Ent(EntRef::new(EntKind::Ray, i))]);
        c.intrinsic = true;
        self.add_quiet(c);
    }

    /// `place_ray` undone: the place held where it stands, and the foot row gone with it.
    fn unplace_ray(&mut self, i: usize) {
        if !self.rays[i].placed {
            return;
        }
        self.rays[i].placed = false;
        for p in self.rays[i].a {
            self.params[p as usize].fixed = true;
        }
        let ray = EntRef::new(EntKind::Ray, i);
        self.constraints.retain(|c| {
            !(c.intrinsic && c.kind == crate::constraints::CKind::RayFoot && c.args[0].ent() == ray)
        });
    }

    /// A fixed Param holding 0, shared by every ray: the kernels that read a line's direction as
    /// `B − A` read a ray's as the segment `(0, d)`, so a ray needs no kernels of its own for
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

    /// A plane over two rays — right along `u`'s, out along `u × v` — standing at `o`, three
    /// Params of its own, and its origin, a point drawn in it and held at `(0, 0)`, minted here.
    /// No intrinsic row: what the plane is, `Sketch::basis` reads off the rays and `o`.
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

    /// A ray as a document or another sketch carries it: its direction, which of the three are
    /// held, and where it stands (`a`, fixed until a relation reads it).
    pub fn ray_restored(&mut self, d: [f64; 3], held: [bool; 3], a: [f64; 3], class: Classes) -> usize {
        let ri = self.ray(d, "");
        for k in 0..3 {
            let (dp, ap) = (self.rays[ri].d[k] as usize, self.rays[ri].a[k] as usize);
            self.params[dp].fixed = held[k];
            self.params[ap].value = a[k];
        }
        self.rays[ri].class = class;
        ri
    }

    /// A plane as a document or another sketch carries it: over rays `u`, `v`, standing at `o`
    /// with which of its three numbers are held, its origin a point already made.
    pub fn plane_restored(&mut self, u: usize, v: usize, o: [f64; 3], held: [bool; 3], origin: usize,
                          class: Classes) -> usize {
        let pi = self.plane_over(u, v, o, origin, "");
        for k in 0..3 {
            let q = self.planes[pi].o[k] as usize;
            self.params[q].fixed = held[k];
        }
        self.planes[pi].class = class;
        pi
    }

    /// The three Params of where a plane stands, `name.x` … `name.z`.
    fn plane_place(&mut self, o: [f64; 3], name: &str) -> [u32; 3] {
        [0, 1, 2].map(|k| self.param(o[k], false, &format!("{name}.{}", ["x", "y", "z"][k])) as u32)
    }

    /// A plane over rays `u`, `v`, standing at `op`, its origin drawn in it.
    fn push_plane(&mut self, u: usize, v: usize, op: [u32; 3], origin: usize) -> usize {
        let pi = self.planes.len();
        self.points[origin].plane = Some(pi as u32);
        self.planes.push(PlaneE { u: u as u32, v: v as u32, o: op, origin: origin as u32, class: Classes::default() });
        pi
    }

    /// A plane fixed where `b` stands — its rays and its origin held — for a caller that has a
    /// basis in hand rather than a document: a test, a drawing's measuring plane.
    pub fn fixed_plane(&mut self, b: crate::plane::Basis, name: &str) -> usize {
        let rays = [(b.u, "u"), (b.v, "v")].map(|(d, k)| {
            let r = self.ray(d, &format!("{name}.{k}"));
            for &p in &self.rays[r].d {
                self.params[p as usize].fixed = true;
            }
            r
        });
        let pi = self.plane(rays[0], rays[1], b.o, name);
        for &p in &self.planes[pi].o {
            self.params[p as usize].fixed = true;
        }
        pi
    }

    /// Stand plane `i` at `o`, its rays untouched — for a caller that holds a sketch and moves a
    /// part rigidly, never for a solve.
    pub fn set_plane_origin(&mut self, i: usize, o: [f64; 3]) {
        for k in 0..3 {
            let p = self.planes[i].o[k] as usize;
            self.params[p].value = o[k];
        }
    }

    /// A cubic B-spline over `ctrl`, with the clamped uniform knot vector.  `None` if there are
    /// too few control points for a cubic — the curve would have no span to live on.
    pub fn spline(&mut self, ctrl: &[usize]) -> Option<usize> {
        self.spline_with(ctrl, None)
    }

    /// A cubic B-spline with a knot vector of its own — a repeated interior knot is a corner.
    /// `None` if the knots are not ones this control polygon can be drawn with.
    pub fn spline_with(&mut self, ctrl: &[usize], knots: Option<Vec<f64>>) -> Option<usize> {
        if ctrl.iter().any(|&c| c >= self.points.len()) {
            return None;
        }
        let knots = knots.unwrap_or_else(|| crate::curve::clamped_uniform(ctrl.len()));
        if !crate::curve::knots_valid(&knots, ctrl.len()) {
            return None;
        }
        self.splines.push(SplineE {
            ctrl: ctrl.iter().map(|&c| c as u32).collect(),
            knots,
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
        id
    }

    /// `add` without the expression pass, for a caller adding a whole document one constraint at
    /// a time and evaluating once at the end — `io::graft`, the rebuild walk behind deletion,
    /// copying, pasting and the part a drag works on.  Evaluating per add would parse every
    /// expression in the document again for each one that carries text, and would make a
    /// dimension whose definition has not been grafted yet briefly a free variable — allocating
    /// an unknown the next pass immediately retires.
    pub(crate) fn add_quiet(&mut self, mut c: Constraint) -> u32 {
        // the twin its planes can feed, decided before anything is minted for it: a projection
        // where either plane moves in the solve reads both images' lifts and both planes' rays,
        // and over two fixed planes only the two drawn points (`CKind::attitude_twin`)
        let reads = c.attitudes_read(self);
        if !reads.is_empty() {
            let solved = reads.iter().any(|&v| !self.plane_fixed(v));
            c.kind = c.kind.attitude_twin(solved);
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
        // a relation that reads where a ray is gives the ray its place
        if let Some(Arg::Ent(r)) = c.kind.place_slot().and_then(|i| c.args.get(i)) {
            if r.kind == EntKind::Ray && r.i() < self.rays.len() {
                self.place_ray(r.i());
            }
        }
        for (i, name) in c.kind.param_slots() {
            if matches!(c.args[i], Arg::Param(_)) {
                continue;   // already allocated (a constraint moved between sketches)
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
        self.constraints.push(c);
        id
    }

    /// Drop a constraint.  Its own unknowns stay in the parameter vector — every index above
    /// them names something — but are retired to `fixed`, since a free parameter no equation
    /// mentions is a degree of freedom the sketch does not actually have, and diagnosis would
    /// report it.  The rebuild walk (`io::without`) is the path that reclaims the slots.
    pub fn remove(&mut self, id: u32) {
        let (expr, own) = self.constraint(id)
            .map_or((false, Vec::new()), |c| (crate::expr::has_expr(&c.args), c.aux_params()));
        let placed = self.constraint(id).and_then(|c| c.ray_placed_by());
        self.constraints.retain(|c| c.id != id);
        // a ray whose place nothing reads now is held again: its place is no freedom
        if let Some(r) = placed {
            if !self.constraints.iter().any(|c| c.ray_placed_by() == Some(r)) {
                self.unplace_ray(r);
            }
        }
        // a shared unknown stays free while another contact still owns it
        for p in own {
            let shared = self.shared.values().any(|s| s.param == p);
            if !shared || !self.constraints.iter().any(|c| c.owns(p)) {
                self.params[p as usize].fixed = true;
            }
        }
        self.placements.remove(&id);
        if expr {
            // it may have defined a name others read, or been the last reader of a free one
            crate::expr::evaluate(self);
        }
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
        self.constraints.iter().filter(|c| !c.claim).map(|c| c.n_residuals()).sum()
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
