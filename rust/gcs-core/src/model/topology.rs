//! Entity children, enumeration and compiled-plan topology keys.

use super::*;

/// The two ends of a drawn edge, as point indices — a line's `p1`/`p2`, an arc's `start`/`end`.
/// `None` for a kind with no ends, which is what a face's loop may not be made of.
pub fn edge_ends(sk: &Sketch, e: EntRef) -> Option<(u32, u32)> {
    match e.kind {
        EntKind::Line => Some((sk.lines.get(e.i())?.p1, sk.lines.get(e.i())?.p2)),
        EntKind::Arc => Some((sk.arcs.get(e.i())?.start, sk.arcs.get(e.i())?.end)),
        // a spline passes through its end control points where its knots are clamped
        EntKind::Spline => {
            let s = sk.splines.get(e.i())?;
            let k = &s.knots;
            let d = crate::curve::DEGREE;
            (s.ctrl.len() > d && k.len() > 2 * d && k[..=d].iter().all(|&x| x == k[0])
                && k[k.len() - 1 - d..].iter().all(|&x| x == k[k.len() - 1]))
                .then(|| (s.ctrl[0], *s.ctrl.last().unwrap()))
        }
        // a curve has ends where a face trimmed it to two points held on it
        EntKind::Curve => sk.curves.get(e.i())?.trim.map(|t| (t.from, t.to)),
        _ => None,
    }
}

/// Entities plus their sub-entities.
pub fn expand(sk: &Sketch, ents: &[EntRef]) -> Vec<EntRef> {
    let mut out = Vec::new();
    for &e in ents {
        out.push(e);
        out.extend(sk.children(e));
    }
    out
}

impl Sketch {
    /// Sub-entities: a line's endpoints, an arc's centre and ends.
    pub fn children(&self, e: EntRef) -> Vec<EntRef> {
        match e.kind {
            EntKind::Point => Vec::new(),
            EntKind::Seam => { let s = &self.seams[e.i()]; vec![s.first,s.second] }
            EntKind::Edge => { let e = &self.edges[e.i()];
                vec![EntRef::new(EntKind::Seam,e.seam as usize),
                    EntRef::new(EntKind::Vertex,e.start as usize),EntRef::new(EntKind::Vertex,e.end as usize),
                    EntRef::line(e.along as usize)] }
            EntKind::Vertex => { let v = &self.vertices[e.i()];
                vec![EntRef::new(EntKind::Seam,v.first as usize),EntRef::new(EntKind::Seam,v.second as usize)] }
            EntKind::Patch => {
                let p = &self.patches[e.i()];
                std::iter::once(p.source).chain(p.inside.iter().chain(&p.outside)
                    .map(|&i| EntRef::solid(i as usize))).collect()
            }
            EntKind::Envelope => {
                let v = &self.envelopes[e.i()];
                vec![EntRef::new(EntKind::Surface,v.surface as usize),
                    EntRef::new(EntKind::Motion,v.motion as usize)]
            }
            // what a motion measures is what it is written over, as much as its axis is
            EntKind::Motion => { let m = &self.motions[e.i()]; match m.def {
                MotionDef::Rotation {axis,..} | MotionDef::Translation {axis,..} => std::iter::once(EntRef::line(axis as usize))
                    .chain(m.measured.iter().flat_map(|x| x.value.ents.values().copied())).collect(),
                MotionDef::Relative {source,observer} => [source,observer].map(|i| EntRef::new(EntKind::Motion,i as usize)).to_vec(),
            } },
            EntKind::Surface => {
                let s = &self.surfaces[e.i()];
                vec![EntRef::solid(s.solid as usize),s.edge]
            }
            // a face's children are the edges it aliases, so deleting one takes the face with
            // it; a solid's are what its term is written over
            EntKind::Face => self.faces[e.i()].on().into_iter().chain(self.faces[e.i()]
                .boundaries().flat_map(|(edges, _)| edges.iter().copied())).collect(),
            EntKind::Solid => {
                let s = &self.solids[e.i()];
                let mut v: Vec<EntRef> = Vec::new();
                match &s.def {
                    SolidDef::Placed { motion, .. } | SolidDef::Swept { motion, .. } => v.push(EntRef::new(EntKind::Motion,*motion as usize)),
                    SolidDef::Prism { face, .. } => v.push(EntRef::face(*face as usize)),
                    SolidDef::Loft { face, end, guide } => {
                        v.push(EntRef::face(*face as usize));
                        v.extend(end.iter().map(|&f| EntRef::face(f as usize)));
                        v.push(*guide);
                    }
                    SolidDef::Through { face, body } => {
                        v.push(EntRef::face(*face as usize));
                        v.push(EntRef::solid(*body as usize));
                    }
                    SolidDef::Revolve { face, axis, .. } => {
                        v.push(EntRef::face(*face as usize));
                        v.push(EntRef::line(*axis as usize));
                    }
                    SolidDef::Body { .. } => {}
                }
                v.extend(s.operands().into_iter().map(|i| EntRef::solid(i as usize)));
                v
            }
            EntKind::Line => {
                let l = &self.lines[e.i()];
                vec![EntRef::point(l.p1 as usize), EntRef::point(l.p2 as usize)]
            }
            EntKind::Circle => vec![EntRef::point(self.circles[e.i()].center as usize)],
            EntKind::Sphere => vec![EntRef::point(self.spheres[e.i()].center as usize)],
            EntKind::Cone | EntKind::Cylinder => vec![EntRef::line(self.axial(e).axis as usize)],
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                vec![
                    EntRef::point(a.center as usize),
                    EntRef::point(a.start as usize),
                    EntRef::point(a.end as usize),
                ]
            }
            EntKind::Spline => {
                self.splines[e.i()].ctrl.iter().map(|&c| EntRef::point(c as usize)).collect()
            }
            EntKind::Plane => {
                let f = self.frame_of(e);
                vec![EntRef::point(f.origin as usize), EntRef::point(f.toward as usize)]
            }
            // the one kind whose children need not be points
            EntKind::Curve => self.curves[e.i()].args.clone(),
        }
    }

    /// The fewest children an entity can still be rebuilt from.  For everything defined by a
    /// fixed set of points that is all of them — a line without an endpoint is nothing.  A
    /// spline is defined by a *list*, so it survives losing one control point while enough are
    /// left to draw a curve with.
    /// Takes the children rather than fetching them, because the one caller has just built the
    /// list to count the survivors in and would otherwise allocate it twice per entity.
    pub fn min_children(&self, e: EntRef, children: &[EntRef]) -> usize {
        // exhaustive on purpose: a new list-shaped entity must stop the build here, or it would
        // inherit the point-shaped answer and be deleted whole instead of shortened
        match e.kind {
            EntKind::Spline => crate::curve::MIN_CTRL,
            // a face is a loop: lose one edge and it is not a loop, so it goes whole.  A solid
            // is its term, and a term missing an operand is not that solid
            EntKind::Point | EntKind::Line | EntKind::Circle | EntKind::Arc | EntKind::Sphere
            | EntKind::Cone | EntKind::Cylinder
            | EntKind::Plane | EntKind::Curve | EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => {
                children.len()
            }
        }
    }

    /// What a compiled plan or `System` depends on: which entities exist, which constraints (by
    /// id, so swapping one Distance for another shows up — counts and type names alone do not)
    /// and which params are fixed.  A cache over compiled artefacts keys on this.
    pub fn topology_key(&self) -> String {
        use std::fmt::Write;
        let mut s = format!(
            "{}|{}|{}|{}|{}|",
            self.points.len(),
            self.lines.len(),
            self.circles.len(),
            self.arcs.len(),
            self.planes.len()
        );
        for sp in &self.splines {
            let _ = write!(s, "s{}:", sp.ctrl.len());
            for k in &sp.knots {
                let _ = write!(s, "{k},");
            }
        }
        s.push('|');
        for c in &self.constraints {
            // a claim compiles to no rows, so claiming a relation and stating it are different
            // topologies even though the constraint list reads the same
            let _ = write!(s, "{}:{}{},", c.id, c.type_name(), if c.claim { "?" } else { "" });
            // A constraint whose columns are not fixed by its entities alone writes them out:
            // which span of a spline a contact sits on, and which unknown a dimension written in
            // terms of a free variable is tied to.  Both are compiled into the plan, so both
            // belong in the key — a contact walking past a knot is a recompile, and so is
            // swapping one free name for another, which leaves the parameter vector exactly as
            // it was.  Not the constants (a dimension's `m` and `c`): a compiled system re-reads
            // those without being rebuilt.
            if c.kind.contact_slots().is_some() || c.free.is_some() {
                for p in c.params(self) {
                    let _ = write!(s, "{p}.");
                }
            }
        }
        s.push('|');
        for p in &self.params {
            s.push(if p.fixed { '1' } else { '0' });
        }
        s
    }

    pub fn count(&self, kind: EntKind) -> usize {
        match kind {
            EntKind::Face => self.faces.len(),
            EntKind::Solid => self.solids.len(),
            EntKind::Surface => self.surfaces.len(),
            EntKind::Motion => self.motions.len(),
            EntKind::Envelope => self.envelopes.len(),
            EntKind::Patch => self.patches.len(),
            EntKind::Seam => self.seams.len(),
            EntKind::Vertex => self.vertices.len(),
            EntKind::Edge => self.edges.len(),
            EntKind::Point => self.points.len(),
            EntKind::Line => self.lines.len(),
            EntKind::Circle => self.circles.len(),
            EntKind::Sphere => self.spheres.len(),
            EntKind::Cone => self.cones.len(),
            EntKind::Cylinder => self.cylinders.len(),
            EntKind::Arc => self.arcs.len(),
            EntKind::Spline => self.splines.len(),
            EntKind::Plane => self.planes.len(),
            EntKind::Curve => self.curves.len(),
        }
    }

    /// Every entity, in creation order per kind.
    pub fn primitives(&self) -> Vec<EntRef> {
        let mut out = Vec::new();
        for kind in [
            EntKind::Point,
            EntKind::Line,
            EntKind::Circle,
            EntKind::Arc,
            EntKind::Spline,
            EntKind::Plane,
            EntKind::Sphere,
            EntKind::Cone,
            EntKind::Cylinder,
        ] {
            for i in 0..self.count(kind) {
                out.push(EntRef::new(kind, i));
            }
        }
        out
    }
}
