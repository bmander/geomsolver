//! What a constraint reads: its constants and its columns, a lift's and an axis's
//! included.

use super::*;

/// A line's two ends lifted, three columns each.
fn lifted_ends(sk: &Sketch, line: usize) -> Vec<u32> {
    let l = &sk.lines[line];
    [l.p1, l.p2]
        .iter()
        .flat_map(|&p| {
            let k = sk.lift_of(p as usize).expect("a relation in space is lifted at the add");
            sk.lifts[k].x
        })
        .collect()
}

/// A plane's columns as every kernel that reads one takes them: its origin, then its two axes'
/// directions — nine Params, held or not.
fn plane_columns(sk: &Sketch, p: usize) -> Vec<u32> {
    let pl = &sk.planes[p];
    [pl.o, sk.axes[pl.u as usize].d, sk.axes[pl.v as usize].d].concat()
}

impl Constraint {
    /// The per-constraint constants the kernel needs (dimension values, chirality flags, and
    /// the local knot window of the span a curve contact sits on).
    pub fn consts(&self, sk: &Sketch) -> Vec<f64> {
        self.consts_on(sk, None)
    }

    /// The same, for a curve contact read on a *given* span — see `params_on`.
    pub fn consts_on(&self, sk: &Sketch, span: Option<usize>) -> Vec<f64> {
        let inner = self.own_consts_on(sk, span);
        let Some(d) = self.along else { return inner };
        // a derivative's: its row's kernel, the row's own constants, then how each of the row's
        // columns moves — held, by a component of the line's direction, or by a tangent column
        // of its own (§6.21)
        let moves = sk.dual_moves(d);
        let mut k = Vec::with_capacity(1 + inner.len() + 8);
        k.push(self.kernel_id() as f64);
        k.extend(inner);
        for p in self.params_on_row(sk, span) {
            k.push(match moves.of(p) {
                crate::model::Move::Held => 0.0,
                crate::model::Move::Dir(c) => (c + 1) as f64,
                crate::model::Move::Tangent => kernels::TANGENT as f64,
            });
        }
        k
    }

    fn own_consts_on(&self, sk: &Sketch, span: Option<usize>) -> Vec<f64> {
        // an envelope contact's: its source and its motion, read off the sketch (#283)
        if let Some(c) = self.kind.envelope_contact() {
            return self.envelope_parts(sk).map(|p| p.consts(c)).unwrap_or_default();
        }
        // a meshing contact's: both flanks' and the mesh's (#283, #345)
        if let Some(c) = self.kind.meshing_contact() {
            return self.meshing_parts(sk).map(|p| p.consts(sk, c.lead())).unwrap_or_default();
        }
        // a contact with a traced body: its values, its home and its body (#326)
        if self.kind.traced_contact().is_some() {
            return self.traced_of().and_then(|e| sk.traced.get(e.i()))
                .map(crate::traced::consts_of).unwrap_or_default();
        }
        // a motion's twist read: its steps (#326)
        if self.kind.motion_reading().is_some() {
            return self.motion_parts(sk).map(|p| crate::envelope::screw::consts(&p.steps))
                .unwrap_or_default();
        }
        if self.kind == CKind::Stationary {
            return crate::variational::pack(sk, self.id);
        }
        // a spline's length reads the whole curve's knots and weights before what it states
        if self.kind == CKind::SplineLength {
            let s = &sk.splines[self.args[0].ent().i()];
            let stated = match self.free {
                Some(f) => vec![f.m, f.c],
                None => vec![self.args[1].num()],
            };
            return crate::integral::length_consts(&s.knots, s.weights.as_deref(), s.ctrl.len(), &stated);
        }
        // a dimension written in terms of a free variable states no number, so what its kernel
        // wants is the map onto the unknown instead: every free twin takes (m, c) and nothing
        // else, which is why this is one branch and not eight
        if let Some(f) = self.free {
            // a skew distance compares a signed gap against the number turned to the seed's
            // side, and turning (m, c) with it keeps the twin's kernel sign-blind; and a word
            // that says which way — `side:`, `along:`, `sense:` — turns them as it turns a
            // stated number below, or `side: right` over a free `k` meant the left
            if self.kind == CKind::LineLine3 {
                let (s, pinned) = self.skew_pin();
                return vec![s * f.m, s * f.c, pinned];
            }
            let s = self.side().unwrap_or(1.0);
            return vec![s * f.m, s * f.c];
        }
        if let Some((sp, t)) = self.spline_contact(sk) {
            let span = span.unwrap_or_else(|| crate::curve::span_of(sk, sp, t));
            let s = &sk.splines[sp];
            let mut k = crate::curve::local_knots(&s.knots, span).to_vec();
            k.extend(crate::curve::local_weights(s.weights.as_deref(), span));
            return k;
        }
        // a curve contact carries its family's compiled body — two tapes, or a whole trace
        // block — and the numbers the instance was given.  They are the same for every contact
        // with the same curve, and duplicated per constraint because that is where a block
        // already has room for numbers — which is what keeps the kernel table `fn`-pointered and
        // ignorant of curves.
        if let Some(curve) = self.curve_of() {
            let cv = &sk.curves[curve.i()];
            let d = &sk.curve_defs[cv.def as usize];
            let mut k = match &d.body {
                crate::model::CurveBody::Exprs { x, y } => {
                    let mut k =
                        Vec::with_capacity(3 + x.flat.len() + y.flat.len() + cv.values.len());
                    k.push(d.vars.len() as f64);
                    k.push(x.flat.len() as f64);
                    k.push(y.flat.len() as f64);
                    k.extend_from_slice(&x.flat);
                    k.extend_from_slice(&y.flat);
                    k.extend_from_slice(&cv.values);
                    k
                }
                // a trace contact adds the march's home — the parameter its instance is
                // anchored at — and, for a curve of a drawn instance, the pose on the sheet
                // the home solve starts from: instance data the way the values are, read off
                // the sketch here so a refresh carries the pose the drawing has now
                crate::model::CurveBody::Trace(l) => {
                    let ci = curve.i();
                    let n_q = l.n_q();
                    let mut k = Vec::with_capacity(3 + cv.values.len() + l.flat.len() + n_q);
                    k.push(sk.curve_home(ci));
                    k.push(cv.values.len() as f64);
                    k.extend_from_slice(&cv.values);
                    match sk.curve_pose(ci) {
                        Some(pose) => {
                            k.push(1.0);
                            k.extend_from_slice(&l.flat);
                            k.extend(pose);
                        }
                        None => {
                            k.push(0.0);
                            k.extend_from_slice(&l.flat);
                            k.extend(std::iter::repeat(0.0).take(n_q));
                        }
                    }
                    k
                }
                // a generated profile carries the roll its root is chosen at, then the
                // numbers its motion was given, then its tool and motion
                crate::model::CurveBody::Envelope(g) => g.contact_consts(sk, curve.i()),
                // a free curve's problem: its Lagrangian and where its pegs are
                crate::model::CurveBody::Extremal(x) => {
                    sk.extremal_consts(curve.i()).unwrap_or_else(|| vec![0.0; x.n_const()])
                }
            };
            // a point in space reads the curve's plane first: its basis in space, `u`, `v`, `o`
            if self.kind == CKind::PointOnExtrusion {
                k.splice(0..0, sk.extrusion_frame(curve.i()));
            }
            return k;
        }
        match self.kind {
            CKind::Distance => vec![self.args[2].num()],
            // signed from the first point to the second, and which way is the word: `along: left`
            // is the minus a drawing used to write (§9.2)
            CKind::Ordinate => vec![self.side().unwrap_or(1.0) * self.args[3].num()],
            // the line kernels a level reads along its view's axes take no number; the others a
            // zero
            CKind::Level => match self.form() {
                Some(OrdinateForm::PageU | OrdinateForm::PageV) => Vec::new(),
                _ => vec![0.0],
            },
            CKind::DragTarget => {
                vec![self.args[1].num(), self.args[2].num(), self.args[3].num()]
            }
            CKind::DragSeen => {
                let (right, up) = crate::overview::eye(self.args[4].num(), self.args[5].num());
                let out = crate::space::cross(right, up);
                [&[self.args[1].num(), self.args[2].num(), self.args[3].num()][..], &right, &up,
                 &out, &[self.args[6].num()]]
                    .concat()
            }
            // the number is a magnitude and the word is its sign: `side: right` is the same
            // statement as the negative used to be, said in a word a reader can check (§9.2)
            CKind::ParallelDistance | CKind::PointLineDistance => {
                vec![self.side().unwrap_or(1.0) * self.args[2].num()]
            }
            // an angle is directed, so `sense: cw` turns the number it states rather than the
            // reader having to write the minus (§9.4)
            CKind::Angle => vec![self.sense() * self.args[2].num()],
            // the sign the second pair's angle is read with: `sense: cw` is its mirror image
            CKind::EqualAngle => vec![self.sense()],
            // the turn the similarity keeps, or with `mirror: true` reverses (#357)
            CKind::MappedSimilar => {
                vec![if matches!(self.args[6], Arg::Bool(true)) { -1.0 } else { 1.0 }]
            }
            CKind::ArcLength | CKind::CurveLength => vec![self.args[1].num()],
            CKind::AnnularDistance => vec![self.args[2].num()],
            CKind::Radius => vec![self.args[1].num()],
            // the word times the radius: the centre stands off the line on the side it names
            CKind::TangentLineCircle => vec![self.side().unwrap_or(1.0)],
            CKind::TangentCircleCircle => {
                vec![if matches!(self.args[2], Arg::Bool(true)) { 1.0 } else { -1.0 }]
            }
            // the fold line in each plane's own coordinates — validated at the add, so a pair
            // with none cannot be here; recomputed with every refresh, being a cross product
            // and four dots, rather than kept in a second skip set beside the curve contacts
            CKind::Project => {
                let basis = |i: usize| sk.basis(self.args[i].ent().i());
                let (a, b) = (basis(2), basis(3));
                let (da, db) = crate::plane::fold_line(&a, &b)
                    .expect("a projection between parallel planes is refused at the add");
                vec![da[0], da[1], db[0], db[1], crate::plane::fold_offset(&a, &b)]
            }
            CKind::Distance3 | CKind::PointLine3 => vec![self.args[2].num()],
            // the magnitude turned to the side the lines stood on when it was stated
            CKind::LineLine3 => {
                let (s, pinned) = self.skew_pin();
                vec![s * self.args[2].num(), pinned]
            }
            CKind::Angle3 => vec![self.args[2].num()],
            // two directions across a line as its hidden points stand now — the first line of a
            // parallel, the line a point is on — refreshed with every `refresh_consts`, so they
            // follow the line between solves
            // — across the second where it is held and the first is not: the rows vanish where
            // the cross product runs along the frame's line, which a held direction it is square
            // to never does, while a line turned far from where it was seeded can (#217)
            CKind::Parallel3 => {
                let across_held = self.direction_held(sk, 1) && !self.direction_held(sk, 0);
                let at = if across_held { 1 } else { 0 };
                let (e1, e2) = across(self.axis_dir(sk, at));
                [e1, e2].concat()
            }
            CKind::PointOnLine3 | CKind::PointOnAxis | CKind::LineOnAxis | CKind::PlaneAxis => {
                let (e1, e2) = across(self.axis_dir(sk, 1));
                [e1, e2].concat()
            }
            CKind::PlaneDistance => vec![self.args[2].num()],
            // two points on the axis a drawing's extent apart, so its two rows weigh alike
            CKind::AxisOnPlane => vec![sk.extent().max(1.0)],
            // two directions across the first axis as it stands now, and a drawing's extent, so
            // its turn and its offset weigh alike
            CKind::AxisCoincident => {
                let (e1, e2) = across(self.axis_dir(sk, 0));
                [e1.to_vec(), e2.to_vec(), vec![sk.extent().max(1.0)]].concat()
            }
            // two directions across the plane's normal as it stands now
            CKind::AxisPerpendicularPlane => {
                let (e1, e2) = across(sk.basis(self.args[1].ent().i()).normal());
                [e1, e2].concat()
            }
            CKind::PlaneParallel => {
                let (e1, e2) = across(sk.basis(self.args[0].ent().i()).normal());
                [e1, e2].concat()
            }
            _ => Vec::new(),
        }
    }

    /// The point a tangency-at-a-contact touches its round entity at: the arc endpoint or the
    /// line endpoint the `at` slot names.  One decode, because `params_on` picks the kernel's
    /// *columns* from it and `cgraph` picks the *cluster element* from it — two readings that
    /// have to name the same point or the plan and the kernel address different geometry.
    pub fn contact_point(&self, sk: &Sketch) -> Option<usize> {
        let at = |i: usize| match &self.args[2] {
            Arg::Str(s) => s.as_str() == ["start", "p1"][i],
            _ => true,
        };
        match self.kind {
            CKind::TangentArcLine => {
                let a = &sk.arcs[self.args[0].ent().i()];
                Some(if at(0) { a.start } else { a.end } as usize)
            }
            CKind::TangentLineCircleAt => {
                let l = &sk.lines[self.args[0].ent().i()];
                Some(if at(1) { l.p1 } else { l.p2 } as usize)
            }
            CKind::TangentCircleAt => Some(self.args[2].ent().i()),
            _ => None,
        }
    }

    /// The spline this constraint touches and the Param holding where on it — `None` for
    /// anything that is not a curve contact, and for one that has not been added yet (its
    /// parameter is still the seed number, not a Param).
    pub fn curve_contact(&self) -> Option<(usize, u32)> {
        let (curve, t) = self.kind.contact_slots()?;
        match self.args[t] {
            Arg::Param(p) => Some((self.args[curve].ent().i(), p)),
            _ => None,
        }
    }

    /// The parametric entity this constraint runs along, of *either* family, and the Param
    /// holding where along it.  `curve_contact` is the spline-only reading, and stays that way
    /// because it gates the span machinery; this is the reading for questions about the
    /// parameter itself — chiefly what one unit of it is worth in world length.
    pub fn parametric_contact(&self) -> Option<(EntRef, u32)> {
        self.contact_at(self.kind.contact_slots())
    }

    /// The entity and the allocated Param at a contact's two slots, once added.
    fn contact_at(&self, slots: Option<(usize, usize)>) -> Option<(EntRef, u32)> {
        let (e, t) = slots?;
        match self.args[t] {
            Arg::Param(p) => Some((self.args[e].ent(), p)),
            _ => None,
        }
    }

    /// Whatever this constraint's place runs along — a spline, a curve, a line's or an arc's run —
    /// and the Param holding where (`CKind::place_on`).
    pub fn place(&self) -> Option<(EntRef, u32)> {
        self.contact_at(self.kind.place_on())
    }

    /// A contact on a curve *family* instance (`PointOnCurve`) and the Param holding where
    /// along it — the third family, asked separately from the other two for the one thing it
    /// shares with a spline and not with an ellipse: a bounded parameter, clamped to the
    /// `over (a, b)` its instance declared (`curve::clamp_contacts`).
    pub fn family_contact(&self) -> Option<(EntRef, u32)> {
        self.contact_at(self.kind.contact_on(SpecKind::Curve))
    }

    /// A contact at a fraction of a line's run or an arc's turn (#248) and the Param holding
    /// the fraction — once added.
    pub fn fraction_contact(&self) -> Option<(EntRef, u32)> {
        self.contact_at(self.kind.fraction_slots())
    }

    /// The spline a curve contact touches and the parameter it currently sits at — also before
    /// `Sketch::add`, while the slot still holds the seed number rather than a Param.
    fn spline_contact(&self, sk: &Sketch) -> Option<(usize, f64)> {
        let (curve, t) = self.kind.contact_slots()?;
        Some((self.args[curve].ent().i(), self.args[t].value(sk)))
    }

    /// The ordered Params the kernel's columns refer to.
    pub fn params(&self, sk: &Sketch) -> Vec<u32> {
        self.params_on(sk, None)
    }

    /// The same, for a curve contact read on a *given* span rather than the one its parameter is
    /// in now.  Which control points the columns name and which knots the constants are is one
    /// choice, not two: `System::new` makes it once and passes it here and to `consts_on`, so a
    /// compiled block cannot end up with one span's columns and another's knots.
    pub fn params_on(&self, sk: &Sketch, span: Option<usize>) -> Vec<u32> {
        let mut ps = self.params_on_row(sk, span);
        // a derivative reads the row's columns, a tangent column for each (the fixed zero where
        // it has none), and the ends of the line it is taken along, in space (§6.21)
        if let Some(d) = self.along {
            let tangents: Vec<u32> = ps.iter().map(|&p| sk.tangent_col(d, p)).collect();
            ps.extend(tangents);
            match sk.duals[d].toward {
                crate::model::Toward::Line(l) => ps.extend(lifted_ends(sk, l)),
                // the axis's direction where a line's ends go, from a held zero
                crate::model::Toward::Axis(a) => {
                    ps.extend([sk.zero_col(); 3]);
                    ps.extend(sk.axes[a].d);
                }
                crate::model::Toward::Chart { .. } => ps.extend([sk.zero_col(); 6]),
            }
        }
        ps
    }

    /// The columns the row reads as itself, before a derivative's — what a use's tangent
    /// columns are minted for (`Sketch::mint_tangents`).
    pub(crate) fn row_params(&self, sk: &Sketch) -> Vec<u32> {
        self.params_on_row(sk, None)
    }

    /// The constants the row reads as itself, before a derivative's.
    pub(crate) fn row_consts(&self, sk: &Sketch) -> Vec<f64> {
        self.own_consts_on(sk, None)
    }

    /// Whether this row may be stated as its derivative under dual `d` (§6.21) — `Ok(false)`
    /// where it reads nothing the use moves, so its derivative is nothing — or why it may not.
    /// The one rule the elaborator and a document reader share.  Asked before the row is added,
    /// so it reads entities, not columns: the dual's point, or geometry of the use's own that
    /// owns a number.
    pub fn differentiable(&self, sk: &Sketch, d: usize) -> Result<bool, String> {
        let dual = &sk.duals[d];
        if let crate::model::Toward::Line(l) = dual.toward {
            if !sk.line_ends(l).iter().all(|&e| sk.has_place(e)) {
                return Err("a line touching a set stands in space, or in a plane: this one is a \
                            2D sketch's"
                    .to_string());
            }
        }
        let what = || crate::model::article(&crate::syntax::snake(self.kind.name()));
        if self.kind.family_kernel().is_some()
            || self.kind.built()
            || !crate::taylor::has_form(self.kernel_id())
        {
            return Err(format!(
                "a set whose body states {} has no derivative yet, so nothing is tangent to it",
                what()
            ));
        }
        // everything the row reads, children and all
        let mut read = self.entities();
        let mut i = 0;
        while i < read.len() {
            let more = sk.children(read[i]);
            read.extend(more.into_iter().filter(|c| c.kind != EntKind::Curve));
            i += 1;
        }
        let point = EntRef::point(dual.point);
        // a point drawn in a view moves in it; a direction solved for in space is read where the
        // point stands, so a row over its place in the view would read it held
        if matches!(dual.toward, crate::model::Toward::Chart { .. })
            && !self.reads_space()
            && read.contains(&point)
            && sk.plane_of(dual.point).is_some()
        {
            return Err(format!(
                "a tangency at a point reads each set where the point stands in space, and {} \
                 reads it in its view",
                what()
            ));
        }
        let moves = |e: &EntRef| {
            *e == point || (dual.owned.contains(e) && !sk.own_params(*e).is_empty())
        };
        Ok(read.iter().any(moves))
    }

    /// The columns the row reads as itself — its own, and a free twin's after them.
    fn params_on_row(&self, sk: &Sketch, span: Option<usize>) -> Vec<u32> {
        let mut ps = self.own_params_on(sk, span);
        // the free column always comes last, so appending it is the whole of what a free twin
        // needs from here — see `expr::Free`
        if let Some(f) = self.free {
            ps.push(f.param);
        }
        ps
    }

    fn own_params_on(&self, sk: &Sketch, span: Option<usize>) -> Vec<u32> {
        let e = |i: usize| self.args[i].ent();
        let pt = |i: usize| sk.point_params(e(i).i()).to_vec();
        let pt_at = |p: u32| sk.point_params(p as usize).to_vec();
        let ln = |i: usize| sk.line_params(e(i).i()).to_vec();
        let centre = |i: usize| sk.point_params(sk.round_center(e(i))).to_vec();
        let rad = |i: usize| sk.round_radius(e(i)) as u32;
        match self.kind {
            CKind::Coincident | CKind::Distance => [pt(0), pt(1)].concat(),
            // the two points as its form reads them: in their view (or the one coordinate a
            // point's ordinate from its view's origin moves), then a line drawn there; in space,
            // their lifts and the direction as a segment, or one lift and the plane's frame
            CKind::Ordinate | CKind::Level => match self.form().unwrap_or(OrdinateForm::Space) {
                OrdinateForm::PageU | OrdinateForm::PageV => [pt(0), pt(1)].concat(),
                OrdinateForm::CoordU => vec![pt(1)[0]],
                OrdinateForm::CoordV => vec![pt(1)[1]],
                OrdinateForm::InView => [pt(0), pt(1), ln(2)].concat(),
                OrdinateForm::Space => {
                    let x = self.lifted_columns(sk);
                    [x[..6].to_vec(), self.direction_columns(sk, 2)].concat()
                }
                // the second point's lift, then the plane's origin and axes: along its normal the
                // plane is the direction, and along its `û` or `v̂` the one the first point is
                // the origin of
                f => {
                    let pl = match f {
                        OrdinateForm::FrameN => e(2).i(),
                        _ => sk.plane_of_origin(e(0).i()).expect("a frame form is from an origin"),
                    };
                    [self.lifted_columns(sk), plane_columns(sk, pl)].concat()
                }
            },
            CKind::Midpoint | CKind::PointOnLine | CKind::PointLineDistance => {
                [pt(0), ln(1)].concat()
            }
            CKind::DragTarget => pt(0),
            // a point in space: its own three numbers, which are where it stands — or a plane's
            // origin, held at its (0, 0), so where the plane stands is the plane's own three
            CKind::DragSeen => {
                let i = e(0).i();
                match (sk.points[i].z, sk.plane_of_origin(i)) {
                    (None, Some(pl)) => sk.planes[pl].o.to_vec(),
                    _ => {
                        let p = &sk.points[i];
                        vec![p.x, p.y, p.z.expect("a point seen in space has its own z")]
                    }
                }
            }
            CKind::Horizontal | CKind::Vertical => ln(0),
            CKind::Parallel
            | CKind::Perpendicular
            | CKind::Angle
            | CKind::ParallelDistance
            | CKind::EqualLength => [ln(0), ln(1)].concat(),
            CKind::EqualAngle => [ln(0), ln(1), ln(2), ln(3)].concat(),
            // every control point: the length reads the whole curve
            CKind::SplineLength => sk.entity_params(e(0)),
            // the curve's own number
            CKind::CurveLength => sk.curves[e(0).i()].length.into_iter().collect(),
            CKind::Stationary => crate::variational::columns(sk, self.id),
            // what its curve's problem reads: the curve's columns, the line's points among them
            // once the curve has the slide
            CKind::CurveTouchesLine => {
                let mut v = sk.entity_params(e(0));
                for p in ln(1) {
                    if !v.contains(&p) {
                        v.push(p);
                    }
                }
                v
            }
            // the centre, the two ends and the radius: the sweep is read off the ends, the
            // length is the radius times it
            CKind::ArcLength => {
                let a = &sk.arcs[e(0).i()];
                [pt_at(a.center), pt_at(a.start), pt_at(a.end), vec![rad(0)]].concat()
            }
            CKind::PointOnCircle => [pt(0), centre(1), vec![rad(1)]].concat(),
            // the point, the line's ends, and the fraction
            CKind::PointAlongLine => [pt(0), ln(1), vec![self.args[2].param()]].concat(),
            // the point, the arc's centre, start and end, its radius, and the fraction
            CKind::PointAlongArc => {
                let a = &sk.arcs[e(1).i()];
                [pt(0), pt_at(a.center), pt_at(a.start), pt_at(a.end), vec![rad(1)],
                    vec![self.args[2].param()]].concat()
            }
            // the hidden points, then the fraction
            CKind::PointAlongLine3 => [self.lifted_columns(sk), vec![self.args[2].param()]].concat(),
            // the point and the centre in space, the radius, the arc's plane's axes, then the
            // arc as drawn there and the fraction
            CKind::PointAlongArc3 => {
                let a = &sk.arcs[e(1).i()];
                let v = self.attitude_read(sk).expect("an arc in space is drawn in a plane");
                [self.lifted_columns(sk), vec![rad(1)], plane_columns(sk, v)[3..].to_vec(),
                    pt_at(a.center), pt_at(a.start), pt_at(a.end), vec![self.args[2].param()]]
                    .concat()
            }
            CKind::Radius => vec![rad(0)],
            CKind::EqualRadius | CKind::AnnularDistance => vec![rad(0), rad(1)],
            CKind::TangentLineCircle => [ln(0), centre(1), vec![rad(1)]].concat(),
            CKind::TangentCircleCircle => {
                [centre(0), vec![rad(0)], centre(1), vec![rad(1)]].concat()
            }
            // both say "the radius is perpendicular to the line at the contact", so both are
            // [contact point, centre, line] — the arc names the line second, the circle first
            CKind::TangentCircleAt => [pt(2), centre(0), ln(1)].concat(),
            CKind::TangentArcLine | CKind::TangentLineCircleAt => {
                let at = self.contact_point(sk).unwrap();
                let line = if self.kind == CKind::TangentArcLine { 1 } else { 0 };
                [sk.point_params(at).to_vec(), centre(if line == 1 { 0 } else { 1 }), ln(line)]
                    .concat()
            }
            CKind::Symmetric => [pt(0), pt(1), ln(2)].concat(),
            // every point as drawn in its page, in spec order: each side is read in its own (#357)
            CKind::Mapped | CKind::MappedSimilar => {
                let n = if self.kind == CKind::Mapped { 8 } else { 6 };
                (0..n).flat_map(pt).collect()
            }
            // the curve columns are one span's control points, which is what keeps the column
            // count fixed however long the spline is
            CKind::PointOnSpline => {
                [pt(0), vec![self.args[2].param()], self.span_params(sk, span)].concat()
            }
            // the point, the parameter it sits at, and every coordinate the curve reads — in
            // `entity_params` order, which is the order the definition's tapes were compiled
            // against, so the gradient that comes back needs no rearranging
            CKind::PointOnCurve => {
                [pt(0), vec![self.args[2].param()], sk.entity_params(e(1))].concat()
            }
            // the point's lift, then the parameter and the curve's coordinates as the contact's
            CKind::PointOnExtrusion => {
                [self.lifted_columns(sk), vec![self.args[2].param()], sk.entity_params(e(1))].concat()
            }
            // the parameter, the curve's coordinates, then what it touches — the frame's
            // column order, so the gradients that come back are the row
            CKind::CurveTangentLine => {
                [vec![self.args[2].param()], sk.entity_params(e(0)), ln(1)].concat()
            }
            // the place the contact holding the point owns, then the tangency's columns: the
            // point is the contact's to hold, and no column here
            CKind::CurveTangentAt => {
                [vec![self.args[3].param()], sk.entity_params(e(0)), ln(1)].concat()
            }
            CKind::SplineTangentAt => {
                [vec![self.args[3].param()], self.span_params(sk, span), ln(1)].concat()
            }
            CKind::CurveCurvature => {
                [vec![self.args[2].param()], sk.entity_params(e(0)), centre(1), vec![rad(1)]].concat()
            }
            // an envelope contact's lead — what it states about — then its place, then the
            // source's and the motion's columns (#283, `envelope::held`)
            CKind::PointOnEnvelope
            | CKind::EnvelopeTangentAt
            | CKind::EnvelopeNormalAt
            | CKind::EnvelopePlaneAt
            | CKind::EnvelopeCurvatureAt => {
                let lift = |p: usize| {
                    let k = sk.lift_of(p).expect("a relation in space is lifted at the add");
                    sk.lifts[k].x.to_vec()
                };
                let (lead, place) = match self.kind {
                    CKind::PointOnEnvelope => (lift(e(0).i()), 2),
                    CKind::EnvelopeTangentAt | CKind::EnvelopeNormalAt => (self.direction_columns(sk, 1), 3),
                    CKind::EnvelopePlaneAt => (plane_columns(sk, e(1).i())[3..].to_vec(), 3),
                    _ => {
                        let c = sk.round_center(e(1));
                        let v = sk.plane_of(c).expect("a circle in space is drawn in a plane");
                        ([lift(c), vec![rad(1)], plane_columns(sk, v)[3..].to_vec()].concat(), 3)
                    }
                };
                let parts = self.envelope_parts(sk).map(|p| p.columns(sk)).unwrap_or_default();
                let at: Vec<u32> = (place..place + 3).map(|i| self.args[i].param()).collect();
                [lead, at, parts].concat()
            }
            // the point's lift, the phase, the lag and the two places, then both flanks' and the
            // mesh's columns (#283, `envelope::meshing`)
            CKind::PointOnMeshing => {
                let k = sk.lift_of(e(0).i()).expect("a relation in space is lifted at the add");
                let own: Vec<u32> = (2..2 + crate::envelope::meshing::UNKNOWNS)
                    .map(|i| self.args[i].param()).collect();
                let parts = self.meshing_parts(sk).map(|p| p.columns(sk)).unwrap_or_default();
                [sk.lifts[k].x.to_vec(), own, parts].concat()
            }
            // a line along the common normal or the path of contact (#345): its direction, then
            // the contact's unknowns and the meshing's columns as the contact's
            CKind::MeshingNormalAt | CKind::MeshingTangentAt => {
                let at: Vec<u32> = (3..3 + crate::envelope::meshing::UNKNOWNS)
                    .map(|i| self.args[i].param()).collect();
                let parts = self.meshing_parts(sk).map(|p| p.columns(sk)).unwrap_or_default();
                [self.direction_columns(sk, 1), at, parts].concat()
            }
            // a contact with a traced body (#326, `traced`): what it states about, the place, then
            // the body's columns
            CKind::PointOnTraced
            | CKind::TracedTangentAt
            | CKind::TracedSurfaceTangentAt
            | CKind::TracedNormalAt
            | CKind::TracedPlaneAt
            | CKind::TracedCurvatureAt => {
                let lift = |p: usize| {
                    let k = sk.lift_of(p).expect("a relation in space is lifted at the add");
                    sk.lifts[k].x.to_vec()
                };
                let (lead, place) = match self.kind {
                    CKind::PointOnTraced => (lift(e(0).i()), 2),
                    CKind::TracedPlaneAt => (plane_columns(sk, e(1).i())[3..].to_vec(), 3),
                    CKind::TracedCurvatureAt => {
                        let c = sk.round_center(e(1));
                        let v = sk.plane_of(c).expect("a circle in space is drawn in a plane");
                        ([lift(c), vec![rad(1)], plane_columns(sk, v)[3..].to_vec()].concat(), 3)
                    }
                    _ => (self.direction_columns(sk, 1), 3),
                };
                let at: Vec<u32> = (place..place + 2).map(|i| self.args[i].param()).collect();
                let body = self.traced_of().and_then(|t| sk.traced.get(t.i()))
                    .map(|t| crate::traced::columns(sk, t)).unwrap_or_default();
                [lead, at, body].concat()
            }
            // a motion's twist read (#326, `envelope::screw`): the line's two lifted ends, or the
            // axis's direction, its place and the pitch, then the motion's axes
            CKind::Reciprocal | CKind::ScrewAxis | CKind::CarriedPoint => {
                let lift = |p: usize| {
                    let k = sk.lift_of(p).expect("a relation in space is lifted at the add");
                    sk.lifts[k].x.to_vec()
                };
                let lead = match self.kind {
                    CKind::Reciprocal => self.direction_columns(sk, 0),
                    CKind::CarriedPoint => {
                        [lift(e(0).i()), lift(e(1).i()), vec![self.args[3].param()]].concat()
                    }
                    _ => {
                        let x = &sk.axes[e(0).i()];
                        [x.d.to_vec(), x.a.to_vec(), vec![self.args[2].param()]].concat()
                    }
                };
                let parts = self.motion_parts(sk).map(|p| p.columns(sk)).unwrap_or_default();
                [lead, parts].concat()
            }
            CKind::SplineTangentLine => {
                [vec![self.args[2].param()], self.span_params(sk, span), ln(1)].concat()
            }
            CKind::SplineCurvature => [
                vec![self.args[2].param()],
                self.span_params(sk, span),
                centre(1),
                vec![rad(1)],
            ]
            .concat(),
            // the two images, drawn in two fixed planes — the fold line is constants
            CKind::Project => [pt(0), pt(1)].concat(),
            // both images' hidden points, then both planes' axes
            CKind::ProjectSolved => {
                let axes = |i: usize| plane_columns(sk, e(i).i())[3..].to_vec();
                [self.lifted_columns(sk), axes(2), axes(3)].concat()
            }
            CKind::AxisUnit => sk.axes[e(0).i()].d.to_vec(),
            // the axis's place and direction, and the plane's origin and axes
            CKind::AxisOnPlane => {
                let r = &sk.axes[e(0).i()];
                [r.a.to_vec(), r.d.to_vec(), plane_columns(sk, e(1).i())].concat()
            }
            CKind::AxisParallelPlane | CKind::AxisPerpendicularPlane => {
                [sk.axes[e(0).i()].d.to_vec(), plane_columns(sk, e(1).i())[3..].to_vec()].concat()
            }
            // each plane's two axis directions
            CKind::PlaneParallel => {
                [plane_columns(sk, e(0).i())[3..].to_vec(), plane_columns(sk, e(1).i())[3..].to_vec()].concat()
            }
            // each axis's place and direction
            CKind::AxisCoincident => {
                let (a, b) = (&sk.axes[e(0).i()], &sk.axes[e(1).i()]);
                [a.a.to_vec(), a.d.to_vec(), b.a.to_vec(), b.d.to_vec()].concat()
            }
            // the first plane's origin and axes, then the second's origin
            CKind::PlaneDistance => {
                [plane_columns(sk, e(0).i()), sk.planes[e(1).i()].o.to_vec()].concat()
            }
            CKind::AxisFoot => {
                let r = &sk.axes[e(0).i()];
                [r.a, r.d].concat()
            }
            // the point's lift, then the axis's place and direction
            CKind::PointOnAxis => {
                let r = &sk.axes[e(1).i()];
                [self.lifted_columns(sk), r.a.to_vec(), r.d.to_vec()].concat()
            }
            // the line's ends lifted, then the axis's place and direction
            CKind::LineOnAxis => {
                let r = &sk.axes[e(1).i()];
                [self.lifted_columns(sk), r.a.to_vec(), r.d.to_vec()].concat()
            }
            // where the plane stands, in the point's place, and the axis's place and direction
            CKind::PlaneAxis => {
                let r = &sk.axes[e(1).i()];
                [sk.planes[e(0).i()].o.to_vec(), r.a.to_vec(), r.d.to_vec()].concat()
            }
            // the hidden point, the point as drawn, and its plane's origin and axes
            CKind::Lift => {
                let l = &sk.lifts[sk.lift_of(e(0).i()).expect("a lift's point has one")];
                [l.x.to_vec(), pt(0), plane_columns(sk, e(1).i())].concat()
            }
            // the hidden points, three columns each, in the order the operands name them
            CKind::Coincident3
            | CKind::Distance3
            | CKind::PointLine3
            | CKind::LineLine3
            | CKind::PointOnLine3
            | CKind::EqualLength3
            | CKind::Midpoint3
            | CKind::Symmetric3 => self.lifted_columns(sk),
            CKind::Angle3 | CKind::Perpendicular3 | CKind::Parallel3 => self.axis_columns(sk),
            // and the plane's origin and axes after them
            CKind::PointOnPlane | CKind::LineOnPlane => {
                [self.lifted_columns(sk), plane_columns(sk, e(1).i())].concat()
            }
            // the point and the centre in space, the radius, and the circle's plane's axes
            CKind::PointOnCircle3 => {
                let v = self.attitude_read(sk).expect("a circle in space is drawn in a plane");
                [self.lifted_columns(sk), vec![rad(1)], plane_columns(sk, v)[3..].to_vec()].concat()
            }
            CKind::Fix | CKind::Ccw | CKind::Cw => {
                unreachable!("{:?} is a gauge and is never in a sketch", self.kind)
            }
        }
    }

    /// The drawn points whose hidden points a relation in space reads, in the order its kernel's
    /// columns take them: a point is itself, a line its two ends, a circle its centre.  Empty for
    /// every kind that reads none.  What `Sketch::add` lifts, and what `validate` asks to be on a
    /// view.
    pub fn lifted_points(&self, sk: &Sketch) -> Vec<usize> {
        if !self.reads_space() {
            return Vec::new();
        }
        let e = |i: usize| self.args[i].ent();
        let ends = |i: usize| {
            let l = &sk.lines[e(i).i()];
            [l.p1 as usize, l.p2 as usize]
        };
        match self.kind {
            CKind::Coincident3 | CKind::Distance3 | CKind::ProjectSolved => {
                vec![e(0).i(), e(1).i()]
            }
            CKind::PointLine3 | CKind::PointOnLine3 | CKind::Midpoint3 | CKind::PointAlongLine3 => {
                [vec![e(0).i()], ends(1).to_vec()].concat()
            }
            CKind::PointOnPlane => vec![e(0).i()],
            // the two points, and a direction that is a drawn line, its ends; over a plane's
            // frame only the second, the first being the frame's origin
            CKind::Ordinate | CKind::Level => {
                if self.form().is_some_and(|f| f.of_frame()) {
                    return vec![e(1).i()];
                }
                let line = (e(2).kind == EntKind::Line).then(|| ends(2).to_vec());
                [vec![e(0).i(), e(1).i()], line.unwrap_or_default()].concat()
            }
            // an axis and a plane, and two planes: in space already
            CKind::AxisOnPlane
            | CKind::AxisCoincident
            | CKind::AxisParallelPlane
            | CKind::AxisPerpendicularPlane
            | CKind::PlaneParallel
            | CKind::PlaneDistance => Vec::new(),
            CKind::LineOnPlane | CKind::LineOnAxis => ends(0).to_vec(),
            CKind::PointOnCircle3 | CKind::PointAlongArc3 => vec![e(0).i(), sk.round_center(e(1))],
            CKind::PointOnExtrusion | CKind::PointOnAxis => vec![e(0).i()],
            // what it states about, then every point its envelope's source and motion read (#283)
            CKind::PointOnEnvelope
            | CKind::EnvelopeTangentAt
            | CKind::EnvelopeNormalAt
            | CKind::EnvelopePlaneAt
            | CKind::EnvelopeCurvatureAt => {
                let lead = match self.kind {
                    CKind::PointOnEnvelope => vec![e(0).i()],
                    CKind::EnvelopeTangentAt | CKind::EnvelopeNormalAt if e(1).kind == EntKind::Line => ends(1).to_vec(),
                    CKind::EnvelopeCurvatureAt => vec![sk.round_center(e(1))],
                    _ => Vec::new(),
                };
                let parts = self.envelope_parts(sk).map(|p| p.lifted()).unwrap_or_default();
                [lead, parts].concat()
            }
            CKind::PointOnMeshing => {
                let parts = self.meshing_parts(sk).map(|p| p.lifted(sk)).unwrap_or_default();
                [vec![e(0).i()], parts].concat()
            }
            CKind::MeshingNormalAt | CKind::MeshingTangentAt => {
                let lead = if e(1).kind == EntKind::Line { ends(1).to_vec() } else { Vec::new() };
                let parts = self.meshing_parts(sk).map(|p| p.lifted(sk)).unwrap_or_default();
                [lead, parts].concat()
            }
            // what it states about, then every point the traced body reads (#326)
            CKind::PointOnTraced
            | CKind::TracedTangentAt
            | CKind::TracedSurfaceTangentAt
            | CKind::TracedNormalAt
            | CKind::TracedPlaneAt
            | CKind::TracedCurvatureAt => {
                let lead = match self.kind {
                    CKind::PointOnTraced => vec![e(0).i()],
                    CKind::TracedCurvatureAt => vec![sk.round_center(e(1))],
                    CKind::TracedPlaneAt => Vec::new(),
                    _ if e(1).kind == EntKind::Line => ends(1).to_vec(),
                    _ => Vec::new(),
                };
                let body = self.traced_of().and_then(|t| sk.traced.get(t.i()))
                    .map(|t| crate::traced::lifted(sk, t)).unwrap_or_default();
                [lead, body].concat()
            }
            // the line's ends, then every point the motion's axes read (#326)
            CKind::Reciprocal | CKind::ScrewAxis | CKind::CarriedPoint => {
                let parts = self.motion_parts(sk).map(|p| p.lifted()).unwrap_or_default();
                let lead = match self.kind {
                    CKind::Reciprocal => ends(0).to_vec(),
                    CKind::CarriedPoint => vec![e(0).i(), e(1).i()],
                    _ => Vec::new(),
                };
                [lead, parts].concat()
            }
            CKind::Symmetric3 => [vec![e(0).i(), e(1).i()], ends(2).to_vec()].concat(),
            // a direction's operands: a line's ends are lifted, and an axis is in space already
            CKind::Angle3 | CKind::Perpendicular3 | CKind::Parallel3 => (0..2)
                .filter(|&i| e(i).kind == EntKind::Line)
                .flat_map(ends)
                .collect(),
            _ => [ends(0), ends(1)].concat(),
        }
    }

    /// The columns of a direction relation's two operands, six each as the kernels read a line
    /// (`B − A`): a line's two hidden points, or an axis's direction as the segment from the
    /// origin, `(0, d)` — so an axis needs no kernels of its own for `parallel`, `perpendicular`
    /// and `angle` (`Sketch::origin_param`).
    fn axis_columns(&self, sk: &Sketch) -> Vec<u32> {
        [self.direction_columns(sk, 0), self.direction_columns(sk, 1)].concat()
    }

    /// Operand `i` as a direction in space, six columns: an axis as the segment from the origin
    /// to its direction, a line as its two ends' hidden points.
    fn direction_columns(&self, sk: &Sketch, i: usize) -> Vec<u32> {
        let e = self.args[i].ent();
        if e.kind == EntKind::Axis {
            let z = sk.zero.expect("an axis mints the origin's Param");
            return [[z, z, z], sk.axes[e.i()].d].concat();
        }
        lifted_ends(sk, e.i())
    }

    /// The axes whose place this reads (`CKind::place_slots`), which `Sketch::add` frees and
    /// `Sketch::remove` holds again once nothing reads them.
    pub fn axes_placed_by(&self) -> impl Iterator<Item = usize> + '_ {
        self.kind.place_slots().iter().filter_map(|&i| match self.args.get(i)? {
            Arg::Ent(r) if r.kind == EntKind::Axis => Some(r.i()),
            _ => None,
        })
    }

    /// Whether every number this reads is held: then it moves nothing.
    pub fn reads_only_held(&self, sk: &Sketch) -> bool {
        self.params(sk).iter().all(|&p| sk.params[p as usize].fixed)
    }

    /// Whether operand `i`'s direction is held: an axis's outright, a line's by its ends, each
    /// held where it is drawn, in a plane held where it stands.
    fn direction_held(&self, sk: &Sketch, i: usize) -> bool {
        let e = self.args[i].ent();
        match e.kind {
            EntKind::Axis => sk.axes[e.i()].d.iter().all(|&p| sk.params[p as usize].fixed),
            EntKind::Line => {
                let l = &sk.lines[e.i()];
                [l.p1, l.p2].iter().all(|&p| {
                    sk.point_held(p as usize)
                        && sk.plane_of(p as usize).is_none_or(|v| sk.plane_fixed(v))
                })
            }
            _ => false,
        }
    }

    /// Operand `i`'s direction as it stands now, as a direction relation reads it.
    fn axis_dir(&self, sk: &Sketch, i: usize) -> [f64; 3] {
        let e = self.args[i].ent();
        if e.kind == EntKind::Axis {
            return sk.axes[e.i()].d.map(|p| sk.params[p as usize].value);
        }
        let l = &sk.lines[e.i()];
        crate::space::sub(sk.lifted(l.p2 as usize), sk.lifted(l.p1 as usize))
    }

    /// Those hidden points' Params, three a point.
    fn lifted_columns(&self, sk: &Sketch) -> Vec<u32> {
        self.lifted_points(sk)
            .into_iter()
            .flat_map(|p| {
                let k = sk.lift_of(p).expect("a relation in space is lifted at the add");
                sk.lifts[k].x
            })
            .collect()
    }

    /// The plane whose attitude this statement's kernel reads, where it reads one: a lift's view,
    /// the plane a point is put on, the view a circle is drawn in.
    pub fn attitude_read(&self, sk: &Sketch) -> Option<usize> {
        match self.kind {
            CKind::Lift | CKind::PointOnPlane | CKind::LineOnPlane => Some(self.args[1].ent().i()),
            CKind::PointOnCircle3 | CKind::PointAlongArc3 => {
                sk.plane_of(sk.round_center(self.args[1].ent()))
            }
            _ => None,
        }
    }

    /// Every plane whose attitude decides which twin this statement is: `attitude_read`'s one,
    /// or a projection's two — which takes the solved twin when either of them is not fixed.
    pub fn attitudes_read(&self, sk: &Sketch) -> Vec<usize> {
        match self.kind {
            CKind::Project | CKind::ProjectSolved => {
                vec![self.args[2].ent().i(), self.args[3].ent().i()]
            }
            _ => self.attitude_read(sk).into_iter().collect(),
        }
    }

    /// A skew distance's `sign` slot as its kernel reads it: the sign its gap carries (1 where
    /// unpinned), and whether it is pinned, as 1 or 0 — unpinned, a magnitude (#217).
    fn skew_pin(&self) -> (f64, f64) {
        let i = self.kind.spec().iter().position(|(n, _)| *n == "sign");
        match i.map(|i| &self.args[i]) {
            Some(Arg::Int(s)) if *s < 0 => (-1.0, 1.0),
            Some(Arg::Int(s)) if *s > 0 => (1.0, 1.0),
            _ => (1.0, 0.0),
        }
    }

    /// The Params of the span given, or of the one this contact currently sits on.
    fn span_params(&self, sk: &Sketch, span: Option<usize>) -> Vec<u32> {
        let (sp, t) = self.spline_contact(sk).expect("not a curve contact");
        sk.spline_span_params(sp, span.unwrap_or_else(|| crate::curve::span_of(sk, sp, t)))
    }
}
