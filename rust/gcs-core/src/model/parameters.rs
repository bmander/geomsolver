//! Parameter ownership, canonical solver order, length scaling and vector access.

use super::*;
use crate::constraints::Arg;
use crate::rng::Rng;

impl Sketch {
    /// The name a shared contact parameter is pinned to (`t == s`), where `p` is one.
    pub fn shared_name(&self, p: u32) -> Option<&str> {
        self.shared.iter().find(|(_, s)| s.param == p).map(|(n, _)| n.as_str())
    }

    /// What an owned unknown travels as into another sketch, before `Sketch::add`: its number,
    /// pinned where it was, or the name it is shared under — the one rule `io::graft` and the
    /// printer's `lift` both read, so a copy and a printed statement own what the original did.
    pub fn owned_arg(&self, p: u32) -> Arg {
        let q = &self.params[p as usize];
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
    /// **An expression's *text* is not rewritten, and the number it came to is converted.**
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
                    // the text stays as written; what it came to converts
                    Some(Arg::Expr(e)) => e.value *= k,
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
            EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => Vec::new(),
            EntKind::Point => self.point_all_params(e.i()),
            EntKind::Circle => vec![self.circles[e.i()].radius],
            EntKind::Cylinder => vec![self.cylinders[e.i()].param],
            // a half-angle is an angle, and a unit's conversion leaves it alone
            EntKind::Cone => Vec::new(),
            // the direction is a unit vector and the place a length
            EntKind::Axis => self.axes[e.i()].a.to_vec(),
            EntKind::Arc => vec![self.arcs[e.i()].radius],
            // where a plane stands is three lengths; which way it faces is its axes'
            EntKind::Plane => self.planes[e.i()].o.to_vec(),
            // a line and a spline are their points, and a curve is its expressions: no number
            // of their own to convert
            EntKind::Line | EntKind::Spline | EntKind::Curve => Vec::new(),
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

    /// One of `e`'s own params as its declaration's `hint(…)` writes it: the value, except a
    /// cone's half-angle, held in radians as every angle the kernels read and written in degrees
    /// as every angle a document states.  What a writeback and a lifted program both spell.
    pub fn seed_value(&self, e: EntRef, p: u32) -> f64 {
        let v = self.params[p as usize].value;
        match e.kind {
            EntKind::Cone => v.to_degrees(),
            // a direction's dust below a double's resolution of a unit vector is 0: `z: 6e-17`
            // written into a source file is a number nobody said
            EntKind::Axis | EntKind::Plane if v.abs() < 1e-12 => 0.0,
            _ => v,
        }
    }

    /// A cone's or a cylinder's axis and the number it owns.
    pub fn axial(&self, e: EntRef) -> &AxialE {
        match e.kind {
            EntKind::Cone => &self.cones[e.i()],
            EntKind::Cylinder => &self.cylinders[e.i()],
            _ => panic!("not a cone or a cylinder"),
        }
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
        Some(names)
    }

    pub fn entity_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            // the stratification, as a table entry: a face and a solid own no parameter, so
            // nothing about either is ever a column of the Jacobian
            EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => Vec::new(),
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
            // the axis's ends, then the number the kind owns
            EntKind::Cone | EntKind::Cylinder => {
                let a = self.axial(e);
                [self.line_params(a.axis as usize).to_vec(), vec![a.param]].concat()
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
            EntKind::Plane => self.planes[e.i()].o.to_vec(),
            // whatever its arguments contribute, in argument order — which is the order its
            // tapes were compiled against and so the order of the Jacobian's columns
            EntKind::Curve => {
                let cv = &self.curves[e.i()];
                let mut v = Vec::new();
                for &a in &cv.args {
                    v.extend(self.entity_params(a));
                }
                // the numbers left unknown, once the expression graph has allocated them
                v.extend(cv.unknowns.iter().filter_map(|n| self.free_vars.get(n).copied()));
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
            EntKind::Cone | EntKind::Cylinder => vec![self.axial(e).param],
            EntKind::Axis => {
                let r = &self.axes[e.i()];
                [r.d, r.a].concat()
            }
            EntKind::Arc => vec![self.arcs[e.i()].radius],
            EntKind::Plane => self.planes[e.i()].o.to_vec(),
            // a curve holds no number of its own: it is its expressions, and they read
            // the geometry rather than owning any; a face and a solid own none for the same
            // reason, one further out — every number of theirs is an extent, an expression the
            // flattener settled, and no solve may write one back
            EntKind::Line
            | EntKind::Spline
            | EntKind::Curve
            | EntKind::Face
            | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => Vec::new(),
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
        crate::expr::sync_free(self);
        true
    }

    pub fn free_indices(&self) -> Vec<i32> {
        self.params
            .iter()
            .enumerate()
            .filter(|(_, p)| !p.fixed)
            .map(|(i, _)| i as i32)
            .collect()
    }

    /// Seeded Gaussian noise on every free parameter (warm starts, witness construction).
    pub fn perturb(&mut self, sigma: f64, seed: u32) {
        let mut rng = Rng::new(seed);
        for p in self.params.iter_mut() {
            if !p.fixed {
                p.value += rng.normal(0.0, sigma) / p.scale;
            }
        }
    }
}
