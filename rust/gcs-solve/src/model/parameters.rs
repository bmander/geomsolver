//! Parameter ownership, canonical solver order, length scaling and vector access.

use super::*;
use crate::constraints::Arg;
use crate::rng::Rng;

impl Sketch {
    /// Hold a face family's running number at `value` (#243 L1, `Sketch::held`), its fixed
    /// parameter made the first time: the parameter.
    pub fn hold(&mut self, name: &str, value: f64, dim: crate::units::Dim) -> u32 {
        if let Some(h) = self.held.get(name) {
            self.params[h.param as usize].value = value;
            return h.param;
        }
        let p = self.param(value, true, &format!("~{name}")) as u32;
        self.held.insert(name.to_string(), crate::model::HeldNumber { param: p, dim, reads: None });
        p
    }

    /// The scalar of the geometry a held number reads (`HeldNumber::reads`), as it stands.
    pub fn held_read(&self, e: EntRef, field: u8) -> Option<f64> {
        use crate::model::{HELD_R, HELD_X, HELD_Y, HELD_Z};
        let value = |p: u32| self.params.get(p as usize).map(|q| q.value);
        match (e.kind, field) {
            (EntKind::Point, HELD_X) => value(self.points.get(e.i())?.x),
            (EntKind::Point, HELD_Y) => value(self.points.get(e.i())?.y),
            (EntKind::Point, HELD_Z) => value(self.points.get(e.i())?.z?),
            (EntKind::Circle | EntKind::Arc, HELD_R) => Some(self.radius_value(e).abs()),
            _ => None,
        }
    }

    /// Every held number that reads the geometry brought up to where that geometry stands.
    pub fn refresh_held(&mut self) {
        let reads: Vec<(u32, f64)> = self.held.values()
            .filter_map(|h| h.reads.and_then(|(e, f)| Some((h.param, self.held_read(e, f)?))))
            .collect();
        for (p, v) in reads {
            self.params[p as usize].value = v;
        }
    }

    /// The numbers held where the drawing stands — a face family's running numbers and the
    /// declared parameters, at the instant held — by name, as a text reading one works it out.
    pub fn held_numbers(&self) -> BTreeMap<String, crate::expr::Aff> {
        self.held.iter()
            .map(|(n, h)| (n.clone(),
                           crate::expr::Aff::of_dim(self.params[h.param as usize].value, h.dim)))
            .collect()
    }

    /// The name a shared contact parameter is pinned to (`t == s`), where `p` is one.
    pub fn shared_name(&self, p: u32) -> Option<&str> {
        self.shared.iter().find(|(_, s)| s.param == p).map(|(n, _)| n.as_str())
    }

    /// What an owned unknown travels as into another sketch, before `Sketch::add`: its number,
    /// pinned where it was, or the name it is shared under — the one rule `graft` and the
    /// printer's `lift` both read, so a copy and a printed statement own what the original did.
    pub fn owned_arg(&self, p: u32) -> Arg {
        let q = &self.params[p as usize];
        // a contact pinned to a face family's running number travels pinned to it by name
        if let Some((name, _)) = self.held.iter().find(|(_, h)| h.param == p) {
            return Arg::Shared { name: name.clone(), seed: Some(q.value) };
        }
        match self.shared_name(p) {
            Some(name) => Arg::Shared { name: name.to_string(), seed: Some(q.value) },
            None => Arg::Seed { value: q.value, pinned: q.fixed },
        }
    }

    /// Every length in the sketch, times `k` — what a paste between two documents in different
    /// units does to the figure it carries.
    ///
    /// **Written out by kind, and exhaustively**, because "is this parameter a length?" is not a
    /// question a `Param` can answer: an axis's direction is a unit vector and a curve's parameter
    /// is a place along it, and scaling either would take the drawing apart.  So each table that
    /// knows says — `own_length_params` per entity kind, `CKind::param_dim` per constraint that
    /// owns an unknown — and a new kind stops the build in the first rather than being silently
    /// left unconverted.
    ///
    /// **An expression's *text* is not rewritten, and the number it came to is converted** —
    /// except a formula of bare numbers (`40 / 2`), which a destination in other units would read
    /// as its own, and which `graft`'s `expr::evaluate` would then write back unconverted: it
    /// is multiplied by the ratio (`(40 / 2) * 25.4`).  A text naming a unit or a name is
    /// already right in either.
    /// `w = 80` is arithmetic the author wrote, and rewriting it would be an edit of what the
    /// document says rather than a change of the units it says it in — the same reason
    /// `commit_seeds` never overwrites `hint(r: Rr)`.  What the formula last came to is not
    /// authored, though: it is a length in the document's units like any other, and it is what a
    /// **free variable** is seeded from, so a figure tied together by `== w` would otherwise
    /// arrive scaled with `w` still at its old size.  Everything that re-evaluates is overwritten
    /// by the `expr::evaluate` at the end of `graft` regardless.
    pub fn rescale(&mut self, k: f64) {
        if k == 1.0 || !k.is_finite() || k <= 0.0 {
            return;
        }
        let mut lengths: Vec<u32> = Vec::new();
        for e in self.primitives() {
            lengths.extend(self.own_length_params(e));
        }
        // a free variable a *length* dimension reads is a length itself, and it is the same
        // unknown however many dimensions read it — so the params are gathered before any is
        // written, and each is scaled once
        for c in &self.constraints {
            let Some(f) = &c.free else { continue };
            if c.dimensions().iter().any(|(_, _, kind)| kind.dim() == crate::units::Dim::LENGTH) {
                lengths.push(f.param);
            }
        }
        // a hidden point's coordinates are lengths no entity owns
        for l in &self.lifts {
            lengths.extend(l.x);
        }
        lengths.sort_unstable();
        lengths.dedup();
        for i in lengths {
            self.params[i as usize].value *= k;
        }
        for c in self.constraints.iter_mut() {
            let reads_a_length =
                c.dimensions().iter().any(|(_, _, kind)| kind.dim() == crate::units::Dim::LENGTH);
            // the offset of `value = m·a + c` is in the dimension's own units, so it converts
            // with the unknown while the ratio `m` does not
            if reads_a_length {
                if let Some(f) = c.free.as_mut() {
                    f.c *= k;
                }
            }
            for (i, (_, kind)) in c.kind.spec().iter().enumerate() {
                let is_length = *kind == crate::constraints::SpecKind::Length
                    || (kind.is_param()
                        && c.kind.param_dim() == Some(crate::units::Dim::LENGTH));
                if !is_length {
                    continue;
                }
                match c.args.get_mut(i) {
                    Some(Arg::Num(v)) => *v *= k,
                    // a Param slot holds a seed on the way in and an index once added; the
                    // index's value was scaled above, the seed is scaled here
                    Some(Arg::Seed { value, .. }) => *value *= k,
                    Some(Arg::Shared { seed: Some(value), .. }) => *value *= k,
                    // what it came to converts; the text stays as written unless it is a
                    // unit-less number, which the destination would read as its own units
                    Some(Arg::Expr(e)) => {
                        e.value *= k;
                        if unitless_number(&e.text, self.units) {
                            e.text = format!("({}) * {k}", e.text);
                        }
                    }
                    _ => {}
                }
            }
        }
        // a callout's placement is two world lengths in a frame that follows the geometry
        // (`callout::Frame`), so it converts with the figure it annotates
        for (t, r) in self.placements.values_mut() {
            *t *= k;
            *r *= k;
        }
    }

    /// Which of an entity's *own* params are lengths.
    ///
    /// Exhaustive for `own_params`' reason, and it is the table `rescale` drives off: a new
    /// entity kind with a number of its own must stop the build here, or a paste between two
    /// documents in different units would leave that number unconverted.
    fn own_length_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Meshing | EntKind::Traced => Vec::new(),
            EntKind::Point => self.point_all_params(e.i()),
            EntKind::Circle => vec![self.circles[e.i()].radius],
            // the direction is a unit vector and the place a length
            EntKind::Axis => self.axes[e.i()].a.to_vec(),
            EntKind::Arc => vec![self.arcs[e.i()].radius],
            // where a plane stands is three lengths; which way it faces is its axes'
            EntKind::Frame => self.frame_params(e.i()),
            // a line and a spline are their points, and a curve is its expressions: no number
            // of their own to convert — but a free curve's length
            EntKind::Line | EntKind::Spline => Vec::new(),
            EntKind::Curve => self.curves[e.i()].length.into_iter().collect(),
        }
    }

    pub fn point_params(&self, i: usize) -> [u32; 2] {
        let p = &self.points[i];
        [p.x, p.y]
    }

    pub fn point_fixed(&self, i: usize) -> bool {
        let p = &self.points[i];
        self.params[p.x as usize].fixed && self.params[p.y as usize].fixed
    }

    pub fn fix_point(&mut self, i: usize, fixed: bool) {
        let (x, y) = (self.points[i].x as usize, self.points[i].y as usize);
        self.params[x].fixed = fixed;
        self.params[y].fixed = fixed;
    }

    pub fn line_params(&self, i: usize) -> [u32; 4] {
        let l = &self.lines[i];
        let (a, b) = (&self.points[l.p1 as usize], &self.points[l.p2 as usize]);
        [a.x, a.y, b.x, b.y]
    }

    /// The Params of the control points one span of a spline reads — `ctrl[span-p ..= span]`,
    /// the only ones whose basis functions are non-zero there, in (x, y) order.  This is what
    /// keeps a contact's column count fixed however long the spline is.
    pub fn spline_span_params(&self, i: usize, span: usize) -> Vec<u32> {
        let s = &self.splines[i];
        let mut v = Vec::with_capacity(2 * crate::curve::SPAN_N);
        for a in 0..crate::curve::SPAN_N {
            let pt = &self.points[s.ctrl[span - crate::curve::DEGREE + a] as usize];
            v.push(pt.x);
            v.push(pt.y);
        }
        v
    }

    /// A point's Params: `x`, `y`, and a point in space's `z`.
    pub fn point_all_params(&self, i: usize) -> Vec<u32> {
        let p = &self.points[i];
        let mut v = vec![p.x, p.y];
        v.extend(p.z);
        v
    }

    /// Radius Param index of a circle or arc — and an ellipse's minor radius, which is what its
    /// one scalar drag resizes.
    pub fn round_radius(&self, e: EntRef) -> usize {
        match e.kind {
            EntKind::Circle => self.circles[e.i()].radius as usize,
            EntKind::Arc => self.arcs[e.i()].radius as usize,
            _ => panic!("not a round entity"),
        }
    }

    /// One of `e`'s own params as its declaration's `hint(…)` writes it: the value, with a
    /// direction's dust below a double's resolution written as 0.  What a writeback and a lifted
    /// program both spell.
    pub fn seed_value(&self, e: EntRef, p: u32) -> f64 {
        let v = self.params[p as usize].value;
        match e.kind {
            // a direction's dust below a double's resolution of a unit vector is 0: `z: 6e-17`
            // written into a source file is a number nobody said
            EntKind::Axis | EntKind::Frame if v.abs() < 1e-12 => 0.0,
            _ => v,
        }
    }


    /// Whether arc `i` was said to turn clockwise and is drawn from its end (`reversed`, #573),
    /// so what reads an end or a fraction of it reads it as written.
    pub fn arc_reversed(&self, i: usize) -> bool {
        self.reversed.contains(&(i as u32))
    }

    /// Whether arc `i` is drawn from the end it was written with: walked so by a chain
    /// (`walked_back`) or said to turn clockwise (`arc_reversed`).
    pub fn drawn_from_end(&self, i: usize) -> bool {
        self.walked_back.contains(&(i as u32)) || self.arc_reversed(i)
    }

    /// Params of any primitive, in the model's canonical order.
    /// The names of an entity's scalars under the name `n`, in `entity_params` order: the kind's
    /// own (`EntKind::scalar_names`), and a point in space's third coordinate, `n.z`, which a
    /// point drawn in a plane does not have.
    pub fn scalar_names(&self, e: EntRef, n: &str) -> Option<Vec<String>> {
        let mut names = e.kind.scalar_names(n)?;
        if e.kind == EntKind::Point && self.points[e.i()].z.is_some() {
            names.push(format!("{n}.z"));
        }
        // an arc drawn from its end — walked so by a chain, or said to turn clockwise — still
        // names its ends as written (#363 S4, #573): its drawn start is `k.end`
        if e.kind == EntKind::Arc && self.drawn_from_end(e.i()) {
            names[2..6].rotate_left(2);
        }
        Some(names)
    }

    pub fn entity_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            // the stratification, as a table entry: a face and a solid own no parameter, so
            // nothing about either is ever a column of the Jacobian
            EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Meshing | EntKind::Traced => Vec::new(),
            EntKind::Point => self.point_all_params(e.i()),
            EntKind::Line => self.line_params(e.i()).to_vec(),
            EntKind::Circle => {
                let c = &self.circles[e.i()];
                let p = &self.points[c.center as usize];
                vec![p.x, p.y, c.radius]
            }
            // its direction, then where it is
            EntKind::Axis => {
                let r = &self.axes[e.i()];
                [r.d, r.a].concat()
            }
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                let mut v = Vec::with_capacity(7);
                for pi in [a.center, a.start, a.end] {
                    let p = &self.points[pi as usize];
                    v.push(p.x);
                    v.push(p.y);
                }
                v.push(a.radius);
                v
            }
            EntKind::Spline => {
                let s = &self.splines[e.i()];
                let mut v = Vec::with_capacity(2 * s.ctrl.len());
                for &c in &s.ctrl {
                    let p = &self.points[c as usize];
                    v.push(p.x);
                    v.push(p.y);
                }
                v
            }
            // where it stands; its axes and its origin point are children of their own
            EntKind::Frame => self.frame_params(e.i()),
            // whatever its arguments contribute, in argument order — which is the order its
            // tapes were compiled against and so the order of the Jacobian's columns
            EntKind::Curve => {
                let cv = &self.curves[e.i()];
                let mut v = Vec::new();
                // a point followed along a parameter names the point it traces, which is the
                // drawing's to place and no column of the curve (#398 A)
                if !self.curve_followed(e.i()) {
                    for &a in &cv.args {
                        v.extend(self.entity_params(a));
                    }
                }
                // the numbers left unknown, once the expression graph has allocated them
                v.extend(cv.unknowns.iter().filter_map(|n| self.free_vars.get(n).copied()));
                // a body that states a rate, drawn, runs from its pose (#339 D2)
                if self.curve_defs[cv.def as usize].home {
                    v.extend(cv.pose.iter().filter_map(|&(e, j)| self.own_param(e, j)));
                }
                // and a free curve's length, its own, then the points of the lines it touches
                v.extend(cv.length);
                for &l in &cv.slides {
                    let ln = &self.lines[l as usize];
                    v.extend(self.point_params(ln.p1 as usize));
                    v.extend(self.point_params(ln.p2 as usize));
                }
                v
            }
        }
    }

    /// The parameters an entity owns *itself*: the ones `entity_params` has that its children do
    /// not.  A point's coordinates, a circle's or an arc's radius, an ellipse's minor — exactly
    /// the `Scalar` fields of `EntKind::fields`, and exactly what a declaration seeds and a solve
    /// may write back into one.
    ///
    /// Exhaustive on purpose, like `min_children`: a new entity kind with a number of its own
    /// must stop the build here, or its number would be a value nothing ever writes down.
    pub fn own_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            EntKind::Point => self.point_all_params(e.i()),
            EntKind::Circle => vec![self.circles[e.i()].radius],
            EntKind::Axis => {
                let r = &self.axes[e.i()];
                [r.d, r.a].concat()
            }
            EntKind::Arc => vec![self.arcs[e.i()].radius],
            EntKind::Frame => self.frame_params(e.i()),
            // a curve holds no number of its own: it is its expressions, and they read
            // the geometry rather than owning any; a face and a solid own none for the same
            // reason, one further out — every number of theirs is an extent, an expression the
            // flattener settled, and no solve may write one back
            EntKind::Line
            | EntKind::Spline
            | EntKind::Curve
            | EntKind::Face
            | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Meshing | EntKind::Traced => Vec::new(),
        }
    }

    /// One of an entity's own scalars — the `j`th of `own_params` — without the list.
    pub fn own_param(&self, e: EntRef, j: usize) -> Option<u32> {
        self.own_params(e).get(j).copied()
    }

    // -- parameter vector ---------------------------------------------------

    pub fn get_x(&self) -> Vec<f64> {
        self.params.iter().map(|p| p.value).collect()
    }

    /// **Every driven deflection's mesh made again where the drawing stands** (#510 S2;
    /// `Sketch::meshed`, `Constraint::made_row_at`), each at the level it was made at, kept where
    /// it meshes; a row that cannot be meshed here keeps the mesh it had.  Whether any was made,
    /// and if so the pose recorded (`meshed_at`).
    pub fn remesh(&mut self) -> bool {
        self.remade(Some)
    }

    /// **Every driving row a filled body is solved coarse on made again twice as fine where the
    /// drawing stands** (#567, `bodies::COARSEST`): what a document solve that holds on a coarse
    /// mesh is solved on from.  Whether any was made, and if so the pose recorded (`meshed_at`);
    /// none where every row is at a reading's own size.
    pub fn refine(&mut self) -> bool {
        self.remade(|held| (held > 1.0).then_some(0.5 * held))
    }

    /// The rows held on `meshed` made again where the drawing stands, each at the level `level`
    /// gives for the level it was made at (none: left as it is); a row that cannot be meshed here
    /// keeps the mesh it had.  Whether any was made, and if so the pose recorded (`meshed_at`).
    fn remade(&mut self, level: impl Fn(f64) -> Option<f64>) -> bool {
        let rows: Vec<_> = self.constraints.iter()
            .filter_map(|c| {
                let k = c.meshed_key(self)?;
                Some((c, k, level(self.meshed.get(&k)?.coarser)?))
            })
            .filter_map(|(c, k, at)| match c.made_row_at(self, at) {
                Ok(crate::constraints::InteriorRow::Meshed(r)) => Some((k, r)),
                _ => None,
            })
            .collect();
        let made = !rows.is_empty();
        self.meshed.extend(rows);
        if made {
            self.meshed_at = Some(self.get_x());
        }
        made
    }

    /// **The drawing's meshes made where another stood** (`meshed_at` of a drawing of the same
    /// source): stood at `x`, meshed again at a reading's own size — where a drive that held ends,
    /// a filled body's levels refined (#567) — and put back where it was.  False, and nothing
    /// changed, for an `x` of another drawing.
    pub fn mesh_at(&mut self, x: &[f64]) -> bool {
        let here = self.get_x();
        if !self.set_x(x) {
            return false;
        }
        self.remade(|_| Some(1.0));
        self.set_x(&here)
    }

    /// Write the parameter vector.  A vector of the wrong length is not this sketch's — writing
    /// the overlapping prefix would scatter one sketch's coordinates over another's — so it is
    /// refused; `false` says nothing was written.
    /// Write a whole parameter vector back — the one seam every solve and every drag comes
    /// through, which is why the dimensions written in terms of a free variable are brought up
    /// to date here: their number is that unknown's, and a reader of the drawing must not be
    /// shown the one it had before the solve moved it.
    pub fn set_x(&mut self, x: &[f64]) -> bool {
        if x.len() != self.params.len() {
            return false;
        }
        for (i, p) in self.params.iter_mut().enumerate() {
            p.value = x[i];
        }
        // which also brings a ring's turned copies up to what they are turns of
        crate::expr::sync_free(self);
        true
    }

    /// The unknowns: every parameter neither held nor derived (a ring's turned copy's, which
    /// `System` folds into the parameters it is worked out from).
    pub fn free_indices(&self) -> Vec<i32> {
        let derived = self.derived_mask();
        let pegged = self.pegged_places();
        self.params
            .iter()
            .enumerate()
            .filter(|&(i, p)| !p.fixed && !derived.get(i).copied().unwrap_or(false) && !pegged.contains(&(i as u32)))
            .map(|(i, _)| i as i32)
            .collect()
    }

    /// Seeded Gaussian noise on every free parameter (warm starts, witness construction).
    pub fn perturb(&mut self, sigma: f64, seed: u32) {
        let mut rng = Rng::new(seed);
        let derived = self.derived_mask();
        let pegged = self.pegged_places();
        for (i, p) in self.params.iter_mut().enumerate() {
            if !p.fixed && !derived.get(i).copied().unwrap_or(false) && !pegged.contains(&(i as u32)) {
                p.value += rng.normal(0.0, sigma) / p.scale;
            }
        }
        self.settle_turns();
    }
}

/// Whether `text` is a closed formula of bare numbers: no name, and no unit written in it.
fn unitless_number(text: &str, units: crate::units::Units) -> bool {
    let Ok(parsed) = crate::expr::parse_in(text, units) else { return false };
    match crate::expr::eval(&parsed.body, &Default::default()) {
        Ok(a) => a.free.is_none() && a.dim.is_scalar(),
        Err(_) => false,
    }
}
