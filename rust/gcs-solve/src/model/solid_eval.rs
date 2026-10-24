//! The sketch's solids made and read: faces and solids added, the body rule's one step and its
//! preview, the evaluated and exact solids and the surfaces supplied for swept ones (memoised
//! in `Sketch::caches`), a prism re-extruded and a revolution re-turned while being sized, and
//! a revolution's axis in its profile's plane.

use super::*;

impl Sketch {
    // -- faces and solids (§6.8, §6.9) --------------------------------------

    /// A face: a loop of edges the document already drew, on the plane they agree about.
    ///
    /// The plane is *read* rather than given, since a face has no points of its own to put
    /// anywhere: it is the one every point of every edge is a member of, and the elaborator has
    /// already refused a loop whose edges disagree (E080).
    pub fn face(&mut self, edges: Vec<EntRef>, names: Vec<String>, name: &str) -> usize {
        let plane = edges
            .first()
            .and_then(|e| self.children(*e).first().copied().or(Some(*e)))
            .and_then(|p| (p.kind == EntKind::Point).then(|| self.plane_of(p.i())).flatten());
        let plane = match plane {
            Some(p) => Some(p),
            None => edges.iter().find_map(|e| {
                self.children(*e).iter().find_map(|c| {
                    (c.kind == EntKind::Point).then(|| self.plane_of(c.i())).flatten()
                })
            }),
        };
        self.faces.push(FaceE {
            edges,
            edge_names: names,
            holes: Vec::new(),
            plane: plane.map(|p| p as u32),
            name: name.to_string(),
            class: Classes::default(),
        });
        self.faces.len() - 1
    }

    /// A solid: a face swept, or a term over other solids.  It allocates no parameter, which is
    /// the whole of what "nothing three-dimensional is ever solved for" comes to in code.
    pub fn solid(&mut self, def: SolidDef, name: &str) -> usize {
        self.solids.push(SolidE {
            def, name: name.to_string(), class: Classes::default(), copy: None, material: None,
            binding: None,
        });
        self.solids.len() - 1
    }

    /// Solid `i` as a body, so the body rule can fill it: a body as it stands, and any other
    /// solid made the body over its own sweep — the name keeps its index, so everything reading
    /// it reads the whole object, and the sweep moves to a stock of the same name past the last,
    /// whose index is returned (`None` where `i` was a body already).
    pub fn as_body(&mut self, i: usize) -> Option<usize> {
        if matches!(self.solids[i].def, SolidDef::Body { .. }) {
            return None;
        }
        let stock = self.solids.len();
        self.solids.push(SolidE { material: None, ..self.solids[i].clone() });
        self.solids[i].def =
            SolidDef::Body { stock: stock as u32, on: Vec::new(), through: Vec::new(), bound: Vec::new() };
        Some(stock)
    }

    /// **`tool word body`, one statement of the body rule** (§6.9): `tool` joins the set of
    /// `body`'s that `word` fills (`body` made a body first, `as_body`).  Refused, with the cause
    /// and leaving the sets as they were, for a solid joined to itself and for a statement that
    /// would close a cycle (E041) — asked once the tool is in, since a `through:` reads only what
    /// its body adds.  A tool `word` already joins to `body` joins nothing more: a set taken away,
    /// added or met twice is the same body (#415's copies, each order of an edge's faces a name for
    /// one solid, say it once for each).  The elaborator states each statement by it and
    /// `preview_body` each tool.
    pub fn join_body(&mut self, body: usize, word: crate::vocab::BodyWord, tool: usize)
        -> Result<(), String> {
        let name = |sk: &Sketch, i: usize| sk.solids[i].name.clone();
        if tool == body {
            return Err(format!("`{0} {1} {0}` joins a solid to itself", name(self, body), word.as_str()));
        }
        self.as_body(body);
        if self.solids[body].def.body_set(word).expect("a body").contains(&(tool as u32)) {
            return Ok(());
        }
        self.redefine(body, |d| d.body_set(word).map(|set| set.push(tool as u32)));
        if crate::solid::reaches(self, body as u32, body as u32) {
            self.solids[body].def.body_set(word).expect("a body").pop();
            return Err(format!("`{}` would be made of itself: `{}` is made of it",
                               name(self, body), name(self, tool)));
        }
        Ok(())
    }

    /// **What the body rule would make of `target`** with `tools` joined to it by `word` (#162
    /// F4), and nothing written: each tool joined by `join_body` — the elaborator's own step, so
    /// the preview is what the statements make and is refused where they would be — on a copy of
    /// the drawing, and the target evaluated there.  The copy keeps what the caches already know
    /// of every operand.  Refused, with the cause, for an index out of range, no tool, a refused
    /// statement and wherever evaluation refuses — a body with a swept operand among them, whose
    /// surface is meshed elsewhere and never for a preview.
    pub fn preview_body(&self, target: usize, word: crate::vocab::BodyWord, tools: &[usize])
        -> Result<std::rc::Rc<crate::solid::EvaluatedSolid>, String> {
        let n = self.solids.len();
        if target >= n || tools.iter().any(|&t| t >= n) {
            return Err("a Boolean to preview names no solid of this drawing".into());
        }
        if tools.is_empty() {
            return Err(format!("nothing to {} `{}`", word.as_str(), self.solids[target].name));
        }
        let mut copy = self.clone();
        for &t in tools {
            copy.join_body(target, word, t)?;
        }
        copy.evaluated_solid(target, crate::solid::ApproximationPolicy::from_unit(0.0))
    }

    /// Solid `i`'s definition changed where it stands by `change`, where it applies (`Some`):
    /// what it then is is no longer what its instance's binding made (#566), so the binding goes.
    pub(crate) fn redefine<R>(&mut self, i: usize, change: impl FnOnce(&mut SolidDef) -> Option<R>)
        -> Option<R> {
        let s = self.solids.get_mut(i)?;
        let changed = change(&mut s.def)?;
        s.binding = None;
        Some(changed)
    }

    /// What the document calls a solid.
    pub fn solid_name(&self, i: usize) -> String {
        self.solids.get(i).map(|s| s.name.clone()).unwrap_or_default()
    }

    /// Validate and evaluate the current pose with an explicit approximation policy.
    /// Different policies coexist; changed geometry/placement discards all old policies.
    pub fn evaluated_solid(
        &self, i: usize, policy: crate::solid::ApproximationPolicy,
    ) -> Result<std::rc::Rc<crate::solid::EvaluatedSolid>, String> {
        let (policy, slot) = self.solid_slot(i, policy);
        // one of a binding's solids its representative's evaluation placed (#566); a refusal of
        // the representative's is asked of this one, so it names its own solid
        if let Some(b) = self.bound(i) {
            if let Ok(rep) = self.evaluated_solid(b.rep, policy) {
                return Ok(self.placed_solid(i, &b, &rep, slot));
            }
        }
        let key = crate::solid::ShapeKey::of(self, i);
        if let Some(found) = self.remembered(slot, &key) {
            return found;
        }
        let value = crate::progress::during(|| crate::progress::Work::evaluate(self, i),
            || crate::solid::EvaluatedSolid::evaluate(self, i, policy).map(std::rc::Rc::new));
        let mut cache = self.caches.solid.borrow_mut();
        cache.retain(|(index, _), old| *index != i || old.key.same_shape(&key));
        super::caches::cap_zoom(&mut cache, i);
        cache.insert(slot, super::caches::SolidSlot { key, built: value.clone(), moved: None });
        value
    }

    /// **Solid `i` as already evaluated** (#412), moved where it has only moved since, and `None`
    /// where it would have to be evaluated: what a gesture asks of a solid it must not build on
    /// every frame.
    pub fn built_solid(&self, i: usize, policy: crate::solid::ApproximationPolicy)
        -> Option<Result<std::rc::Rc<crate::solid::EvaluatedSolid>, String>> {
        let (policy, slot) = self.solid_slot(i, policy);
        // one of a binding's placed copies is built wherever its representative is (#566)
        if let Some(b) = self.bound(i) {
            if let Some(Ok(rep)) = self.built_solid(b.rep, policy) {
                return Some(Ok(self.placed_solid(i, &b, &rep, slot)));
            }
        }
        self.remembered(slot, &crate::solid::ShapeKey::of(self, i))
    }

    /// The approximation solid `i` is evaluated at when `policy` is asked, and the slot of
    /// `Caches::solid` it is remembered in.  A swept solid's field mesh does not depend on the
    /// view's pixel length: one mesh serves every zoom, rather than a refinement per wheel tick.
    fn solid_slot(&self, i: usize, policy: crate::solid::ApproximationPolicy)
        -> (crate::solid::ApproximationPolicy, (usize, (u8, u64))) {
        let policy = if crate::solid::has_sweep(self, i) { crate::solid::ApproximationPolicy::Mesh } else { policy };
        (policy, (i, policy.cache_key()))
    }

    /// What `Caches::solid` holds for `slot` against `key`: the evaluation itself where the solid
    /// stands where it was built, and moved where it is the same shape elsewhere (#412).
    fn remembered(&self, slot: (usize, (u8, u64)), key: &crate::solid::ShapeKey)
        -> Option<Result<std::rc::Rc<crate::solid::EvaluatedSolid>, String>> {
        let mut cache = self.caches.solid.borrow_mut();
        let found = cache.get_mut(&slot).filter(|f| f.key.same_shape(key))?;
        if found.key.same_pose(key) {
            return Some(found.built.clone());
        }
        // an evaluation refused is asked again where it now stands: a refusal may be of the place
        let built = found.built.as_ref().ok()?;
        let pose = key.pose();
        if let Some((at, solid)) = &found.moved {
            if *at == pose { return Some(Ok(solid.clone())); }
        }
        let motion = crate::solid::Rigid::between(&found.key.pose(), &pose);
        let solid = std::rc::Rc::new(built.moved(&motion));
        found.moved = Some((pose, solid.clone()));
        Some(Ok(solid))
    }

    /// **Solid `i`, bound as `b`, as its representative's evaluation `rep` placed** (#566): the
    /// representative's shape renamed once for as long as it is that shape, and placed again
    /// only where the representative's evaluation or the motion has changed.
    fn placed_solid(&self, i: usize, b: &super::Bound, rep: &std::rc::Rc<crate::solid::EvaluatedSolid>,
                    slot: (usize, (u8, u64))) -> std::rc::Rc<crate::solid::EvaluatedSolid> {
        let mut cache = self.caches.placed.borrow_mut();
        if let Some(hit) = cache.get(&slot).and_then(|(_, p)| p.hit(rep, &b.motion)) {
            return hit;
        }
        let mut renamed = match cache.remove(&slot) {
            Some((renamed, _)) => renamed,
            None => rep.renamed(&b.from, &b.to),
        };
        let placed = std::rc::Rc::new(rep.placed(&b.motion, &mut renamed, &b.from, &b.to));
        super::caches::cap_zoom(&mut cache, i);
        let placement = super::caches::Placement { of: rep.clone(), motion: b.motion, placed: placed.clone() };
        cache.insert(slot, (renamed, placement));
        placed
    }

    /// Solid `i`'s exact B-rep (`solid::Exact`), or why this kernel does not build it: built once
    /// for the geometry it reads, whatever approximation asks — and once for its binding (#566),
    /// a solid of one made by another instance being the representative's placed.
    pub fn exact_solid(&self, i: usize) -> Result<std::rc::Rc<crate::solid::Exact>, String> {
        if let Some(b) = self.bound(i) {
            if let Ok(rep) = self.exact_solid(b.rep) {
                let mut cache = self.caches.placed_exact.borrow_mut();
                if let Some(hit) = cache.get(&i).and_then(|p| p.hit(&rep, &b.motion)) {
                    return Ok(hit);
                }
                let placed = std::rc::Rc::new(rep.placed(&b.motion, &b.from, &b.to));
                cache.insert(i, super::caches::Placement { of: rep, motion: b.motion, placed: placed.clone() });
                return Ok(placed);
            }
        }
        let key = crate::solid::MemoKey::of(self, i);
        let build = || crate::progress::during(|| crate::progress::Work::build(self, i),
            || crate::solid::build_exact(self, i).map(std::rc::Rc::new));
        self.caches.exact.get_or(i, key, build)
    }

    /// Give swept solid `i` its exact B-rep, built elsewhere (`brep::export::supply_exact`) against
    /// the drawing as it stands now: its evaluation then reads the B-rep, as a static solid's does
    /// (`EvaluatedSolid::from_brep`), until the drawing changes.
    pub fn supply_exact_solid(&self, i: usize, exact: crate::solid::Exact) {
        self.caches.exact.put(i, crate::solid::MemoKey::of(self, i), Ok(std::rc::Rc::new(exact)));
    }

    /// The exact B-rep supplied for swept solid `i` against the drawing as it stands, if any.
    pub fn supplied_exact_solid(&self, i: usize) -> Option<std::rc::Rc<crate::solid::Exact>> {
        self.caches.exact.get(i, &crate::solid::MemoKey::of(self, i)).and_then(Result::ok)
    }

    /// **What a host meshing swept solids elsewhere has to mesh** (`FieldMeshing::Deferred`): the
    /// objects (`model::objects`) with a continuous sweep among their operands, each with the
    /// key of the drawing it is a surface of (`FieldJob`). A swept solid that is no object is left
    /// out: nothing shows or exports it, so its surface is never asked for.
    pub fn field_jobs(&self) -> Vec<crate::solid::FieldJob> {
        super::objects(self).into_iter().filter(|&i| self.is_swept(i))
            .map(|i| crate::solid::FieldJob {
                solid: i, name: self.solids[i].name.clone(), key: crate::solid::field_key(self, i),
            })
            .collect()
    }

    /// Give swept solid `i` a surface meshed elsewhere, against the drawing as it stands now.
    /// Refused, with the reason, for an index out of range, a solid with no sweep (whose surface
    /// is its own) or a triangle naming a vertex that is not there — a surface crosses from
    /// another host, and the core checks what it is handed.
    pub fn supply_field(&self, i: usize, surface: crate::solid::FieldSurface) -> Result<(), String> {
        if i >= self.solids.len() { return Err("a supplied surface names no solid of this drawing".into()); }
        if !self.is_swept(i) {
            return Err(format!("`{}`: a supplied surface is for a swept solid, and this one has no sweep",
                self.solids[i].name));
        }
        let n = surface.vertices.len();
        if surface.triangles.iter().flatten().any(|&v| v as usize >= n) {
            return Err(format!("`{}`: a supplied triangle names a vertex past the {n} given", self.solids[i].name));
        }
        // an exact surface is a finished one, and says the face of every triangle
        if let Some(x) = &surface.exact {
            let name = &self.solids[i].name;
            if surface.provisional { return Err(format!("`{name}`: an exact surface is never provisional")); }
            if x.of.len() != surface.triangles.len() {
                return Err(format!("`{name}`: an exact surface names the faces of {} of its {} triangles", x.of.len(),
                    surface.triangles.len()));
            }
            if x.of.iter().any(|&f| f as usize >= x.smooth.len()) {
                return Err(format!("`{name}`: an exact surface's triangle names a face past the {} given", x.smooth.len()));
            }
            if !(x.volume.is_finite()) { return Err(format!("`{name}`: an exact surface's volume is not a number")); }
        }
        let key = crate::solid::reads(self, i, 0.0);
        self.field_surfaces.borrow_mut().insert(i, (key, std::rc::Rc::new(surface)));
        self.caches.solid.borrow_mut().retain(|(index, _), _| *index != i);
        Ok(())
    }

    /// Whether solid `i`'s surface is still being refined elsewhere: false for a solid with no
    /// sweep, answered without evaluating it, and a swept one's supplied surface's word otherwise.
    pub fn field_provisional(&self, i: usize) -> Result<bool, String> {
        if i >= self.solids.len() { return Err("no such solid in this drawing".into()); }
        if !self.is_swept(i) { return Ok(false); }
        // any policy: a swept solid is always its supplied mesh, whatever the unit asked
        self.evaluated_solid(i, crate::solid::ApproximationPolicy::from_unit(0.)).map(|s| s.provisional())
    }

    /// The surface supplied for swept solid `i`, while the drawing still reads as it did.
    pub fn supplied_field(&self, i: usize) -> Option<std::rc::Rc<crate::solid::FieldSurface>> {
        let supplied = self.field_surfaces.borrow();
        let (key, surface) = supplied.get(&i)?;
        (*key == crate::solid::reads(self, i, 0.0)).then(|| surface.clone())
    }

    /// Whether solid `i` has a continuous sweep among its operands, and so a field's surface.
    pub fn is_swept(&self, i: usize) -> bool {
        crate::solid::has_sweep(self, i)
    }

    /// Compatibility output in world coordinates. New queries use `evaluated_solid` so
    /// invalid geometry remains a diagnostic instead of an empty drawing.
    pub fn solid_boundary(&self, i: usize, unit: f64) -> Vec<crate::csg::Piece> {
        self.evaluated_solid(i, crate::solid::ApproximationPolicy::from_unit(unit))
            .map(|s| s.world_boundary()).unwrap_or_default()
    }

    pub fn solid_mesh(&self, i: usize, unit: f64) -> crate::mesh::Mesh {
        self.evaluated_solid(i, crate::solid::ApproximationPolicy::from_unit(unit))
            .map(|s| s.world_mesh()).unwrap_or_else(|_| crate::mesh::grouped(&[]))
    }

    pub fn solid_edges(&self, i: usize, unit: f64) -> Vec<crate::csg::Edge> {
        self.evaluated_solid(i, crate::solid::ApproximationPolicy::from_unit(unit))
            .map(|s| s.world_edges()).unwrap_or_default()
    }
}

/// Which way an extrusion runs from its face, and how thick it is the language's way to say: behind
/// it (`depth: d`, the material the view looks at), in front of it (`from: 0, to: d`), or both
/// ways (`from: -d/2, to: d/2`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Way {
    Behind,
    Front,
    Both,
}

impl Way {
    /// Where an extrusion `d` thick runs along its face's normal, from and to.
    pub fn extents(self, d: f64) -> (f64, f64) {
        match self {
            Way::Behind => (-d, 0.0),
            Way::Front => (0.0, d),
            Way::Both => (-d / 2.0, d / 2.0),
        }
    }

    /// The way and thickness a prism's extents say, where they say one of the three.
    pub fn of(from: f64, to: f64) -> Option<(Way, f64)> {
        if to == 0.0 && from < 0.0 {
            Some((Way::Behind, -from))
        } else if from == 0.0 && to > 0.0 {
            Some((Way::Front, to))
        } else if from == -to && to > 0.0 {
            Some((Way::Both, 2.0 * to))
        } else {
            None
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Way::Behind => "behind",
            Way::Front => "front",
            Way::Both => "both",
        }
    }

    pub fn parse(w: &str) -> Option<Way> {
        [Way::Behind, Way::Front, Way::Both].into_iter().find(|x| x.as_str() == w)
    }
}

impl Sketch {
    /// Re-extrude a prism where it stands, `d` thick the `way` it runs — the preview an
    /// extrusion being sized shows before the source says it (#162 F2).  Evaluated solids are
    /// memoised against every extent they read, so the next mesh is re-cut, and `shape_epoch`
    /// says so to a binding.  False for a solid that is not a prism.
    pub fn set_prism(&mut self, i: usize, way: Way, d: f64) -> bool {
        let (from, to) = way.extents(d);
        let set = self.redefine(i, |d| match d {
            SolidDef::Prism { from: f, to: t, .. } => {
                *f = Extent::at(from);
                *t = Extent::at(to);
                Some(())
            }
            _ => None,
        });
        if set.is_some() {
            self.shape_epoch = self.shape_epoch.wrapping_add(1);
        }
        set.is_some()
    }

    /// Re-turn a revolution where it stands through `degrees`, signed (positive right-handed
    /// about its axis, `turn_of`'s reading) — the preview a revolution being sized shows before
    /// the source says it (#162 F3), as `set_prism` is an extrusion's.  False for a solid that is
    /// not one, or no turn.
    pub fn set_turn(&mut self, i: usize, degrees: f64) -> bool {
        let Some((how_far, sense)) = turn_of(degrees) else { return false };
        let set = self.redefine(i, |d| match d {
            SolidDef::Revolve { sweep, sense: s, .. } => {
                *sweep = match how_far {
                    None => Extent { text: String::new(), value: std::f64::consts::TAU },
                    Some(d) => Extent { text: crate::vocab::num(d), value: d.to_radians() },
                };
                *s = sense;
                Some(())
            }
            _ => None,
        });
        if set.is_some() {
            self.shape_epoch = self.shape_epoch.wrapping_add(1);
        }
        set.is_some()
    }
}

/// A turn said as signed degrees, positive right-handed about its axis, as the language says one:
/// how far in degrees — `None` for a whole turn, at 360 or past it, which a revolution writes as
/// no `sweep:` — and which way.  `None` for no turn at all.  The one reading, shared by the
/// preview (`Sketch::set_turn`) and the edit (`edit::set_turn`).
pub fn turn_of(degrees: f64) -> Option<(Option<f64>, Sense)> {
    if !degrees.is_finite() || degrees == 0.0 {
        return None;
    }
    let sense = if degrees < 0.0 { Sense::Cw } else { Sense::Ccw };
    Some(((degrees.abs() < 360.0).then_some(degrees.abs()), sense))
}

/// How far off its profile's plane a revolution's axis may lie and still be read as in it: a
/// direction's sine, and a place's distance over the drawing's reach.  Coarser than `views`'
/// `ON_PLANE`, which judges held datums: this one judges an axis a solve may have placed.
const AXIS_IN_PLANE: f64 = 1e-7;

impl Sketch {
    /// A revolution's axis in its profile's own plane coordinates, from and toward: a line's two
    /// points as drawn there, or an axis's foot and one unit along it, projected — `None` where
    /// an axis does not lie in the plane (a line is held to it by membership, at elaboration).
    pub fn revolve_axis(&self, face: u32, axis: EntRef) -> Option<((f64, f64), (f64, f64))> {
        match axis.kind {
            EntKind::Line => {
                let l = self.lines.get(axis.i())?;
                Some((self.point_xy(l.p1 as usize), self.point_xy(l.p2 as usize)))
            }
            EntKind::Axis => self.axis_in(self.faces.get(face as usize)?.plane, axis.i()),
            _ => None,
        }
    }

    /// Axis `ax` in plane `plane`'s own coordinates (`None` the page), from its foot toward one
    /// unit along it — `None` where it does not lie in the plane.
    pub fn axis_in(&self, plane: Option<u32>, ax: usize) -> Option<((f64, f64), (f64, f64))> {
        use crate::space::{add, dot, normalised, sub};
        let x = self.axes.get(ax)?;
        let value = |q: [u32; 3]| q.map(|q| self.params[q as usize].value);
        let (d, a) = (value(x.d), value(x.a));
        let basis = match plane {
            Some(p) => { self.frames.get(p as usize)?; self.basis(p as usize) }
            None => crate::plane::Basis::page(),
        };
        let n = basis.normal();
        let unit = normalised(d)?;
        if dot(unit, n).abs() > AXIS_IN_PLANE
            || dot(sub(a, basis.o), n).abs() > AXIS_IN_PLANE * self.extent() {
            return None;
        }
        Some((basis.view_coords(a), basis.view_coords(add(a, unit))))
    }
}
