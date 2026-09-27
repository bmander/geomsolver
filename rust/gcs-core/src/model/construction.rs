//! Geometry construction and constraint mutation, preserving parameter and identity order.

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
    if cross.abs() <= tol * ux.hypot(uy) * vx.hypot(vy) {
        return None;
    }
    let d = 2.0 * cross;
    let (u2, v2) = (ux * ux + uy * uy, vx * vx + vy * vy);
    let ox = ax + (vy * u2 - uy * v2) / d;
    let oy = ay + (ux * v2 - vx * u2) / d;
    let r = (ax - ox).hypot(ay - oy);
    let ta = (ay - oy).atan2(ax - ox);
    let tb = (by - oy).atan2(bx - ox);
    let tau = 2.0 * std::f64::consts::PI;
    let sweep = |th: f64| ((th - ta) % tau + tau) % tau;
    let to_b = sweep(tb);
    let to_c = sweep((cy - oy).atan2(cx - ox));
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
        self.points.push(PointE { x: px as u32, y: py as u32, plane: None });
        self.points.len() - 1
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

    /// An arc plus its two intrinsic `PointOnCircle` constraints.
    pub fn arc(&mut self, center: usize, start: usize, end: usize, name: &str) -> usize {
        let (cx, cy) = self.point_xy(center);
        let (sx, sy) = self.point_xy(start);
        let r = (sx - cx).hypot(sy - cy);
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

    /// A plane: a frame with a stated attitude in space (`plane::Basis`), and the same two
    /// intrinsics.  The basis arrives resolved — the model stores what a plane *is*, and refusing
    /// a degenerate one is the job of whoever read it (`Basis::explicit`).
    pub fn plane(
        &mut self,
        origin: usize,
        toward: usize,
        basis: crate::plane::Basis,
        name: &str,
    ) -> usize {
        let frame = self.datum(origin, toward, name);
        self.planes.push(PlaneE { frame, basis, att: None });
        let pi = self.planes.len() - 1;
        self.slave(EntRef::plane(pi));
        pi
    }

    /// Plane `i`'s attitude in space.  The one reader: every consumer outside the model asks
    /// here and never reads the field, so an attitude that comes to be solved for rather than
    /// stated changes this function and no caller (`docs/spatial-constraints-plan.md`).
    ///
    /// A **solved** view (`att`) is read off its unknowns: `u = R(q)·e₁`, `v = R(q)·e₂` and
    /// `o = R(q)·(a, b, d)`, with `R` the rotation of `q / |q|` — so a `q` a solve has not yet
    /// brought back to the unit sphere still reads as an orthonormal basis.  While the unknowns
    /// hold exactly the numbers they were minted at (`Att::seat`) the stored basis is the answer,
    /// which is what makes freeing a view move nothing.
    pub fn basis(&self, i: usize) -> crate::plane::Basis {
        let p = &self.planes[i];
        let Some(a) = &p.att else { return p.basis };
        let now = self.att_values(a);
        if now.iter().zip(&a.seat).all(|(x, y)| x.to_bits() == y.to_bits()) {
            return p.basis;
        }
        let q = [now[0], now[1], now[2], now[3]];
        match crate::plane::quat_rotate(q, [a.ab[0], a.ab[1], now[4]]) {
            Some(o) => crate::plane::from_quat(q, o).unwrap_or(p.basis),
            // a quaternion of no length names no attitude: the last one stated stands
            None => p.basis,
        }
    }

    /// Stand plane `i`'s origin at `o`, its directions untouched — the one writer after
    /// elaboration built the plane, which is what `against` and a derived offset do.  A solved
    /// view is re-seated on the new basis, so its unknowns say the same thing.
    pub fn set_plane_origin(&mut self, i: usize, o: [f64; 3]) {
        self.planes[i].basis.o = o;
        self.seat_attitude(i);
    }

    /// Replace plane `i`'s whole stated attitude — for a caller that holds a sketch and turns
    /// its views in space (the tests that move a part rigidly), never for a solve.
    pub fn set_basis(&mut self, i: usize, b: crate::plane::Basis) {
        self.planes[i].basis = b;
        self.seat_attitude(i);
    }

    /// The rotor's two params, seeded from the chord — the half of `frame` a plane shares.
    fn datum(&mut self, origin: usize, toward: usize, name: &str) -> FrameE {
        let ((c, s), scale) = self.frame_chord(origin, toward);
        let cp = self.param_scaled(c, false, &format!("{name}.c"), scale);
        let sp = self.param_scaled(s, false, &format!("{name}.s"), scale);
        FrameE {
            origin: origin as u32,
            toward: toward as u32,
            c: cp as u32,
            s: sp as u32,
            class: Classes::default(),
        }
    }

    /// The two intrinsics that hold a datum's rotor to its chord — minted here and nowhere
    /// else, since intrinsics are never serialized.
    fn slave(&mut self, e: EntRef) {
        let c1 = Constraint::frame_unit(e);
        let c2 = Constraint::frame_align(self, e);
        self.add(c1);
        self.add(c2);
    }

    /// The rotor of the chord `origin → toward`, and the world length one unit of it is worth.
    /// A coincident pair names no direction, so it reads as the identity rotor at unit scale —
    /// the raw delta would normalise to (0, 0) and start life violating `frame_unit`.
    ///
    /// The one answer, so the rotor's `Param::scale` and the seed of the alignment's own unknown
    /// cannot come from two different rules — the same reason `constraints::contact_speed` is
    /// one function.
    pub(crate) fn frame_chord(&self, origin: usize, toward: usize) -> ((f64, f64), f64) {
        let (ox, oy) = self.point_xy(origin);
        let (tx, ty) = self.point_xy(toward);
        let d = (tx - ox).hypot(ty - oy);
        if d > 0.0 { (((tx - ox) / d, (ty - oy) / d), d) } else { ((1.0, 0.0), 1.0) }
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
        if c.id == 0 {
            self.next_cid += 1;
            c.id = self.next_cid;
        } else {
            self.next_cid = self.next_cid.max(c.id);
        }
        let id = c.id;
        for (i, name) in c.kind.param_slots() {
            if matches!(c.args[i], Arg::Param(_)) {
                continue;   // already allocated (a constraint moved between sketches)
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
        let mut expr = false;
        if let Some(c) = self.constraint(id) {
            expr = crate::expr::has_expr(&c.args);
            for p in c.aux_params() {
                self.params[p as usize].fixed = true;
            }
        }
        self.constraints.retain(|c| c.id != id);
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
