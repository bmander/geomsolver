//! Spatial entity storage, construction and evaluated-solid cache access.

use super::*;

/// An inner boundary and the source names of its swept sides.
#[derive(Clone, Debug)]
pub struct FaceLoop {
    pub edges: Vec<EntRef>,
    pub edge_names: Vec<String>,
}

/// A face's support is either an inherited profile plane or a named spatial
/// surface/envelope. Spatial support must never default to the page plane.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum FaceSupport { Plane(Option<u32>), Surface(EntRef) }

/// A region bounded by existing edges. Planar profiles inherit their plane from
/// point membership; spatial faces name their analytic support explicitly.
#[derive(Clone, Debug)]
pub struct FaceE {
    /// The loop, in traversal order: planar lines/arcs/circles, or named spatial edges.
    pub edges: Vec<EntRef>,
    /// What the document calls each edge, in the same order — the name the face swept from it
    /// is reached by (`block.side_l`).  Kept beside the references because a solid's face path
    /// is spelled out of the *source's* words, and an anonymous edge has none to spell.
    pub edge_names: Vec<String>,
    /// Strictly contained, disjoint inner boundaries.
    pub holes: Vec<FaceLoop>,
    /// Inherited profile plane (`None` is the page) or exact named spatial support.
    pub support: FaceSupport,
    /// What the document calls it.  A solid's faces are reached by *path* — `body.bore.wall` —
    /// so unlike every other kind a face and a solid carry their own name: the path is the
    /// naming mechanism, and a report that could not spell one would have nothing to say.
    pub name: String,
    pub class: Classes,
}

impl FaceE {
    pub fn plane(&self) -> Result<Option<u32>,String> {
        match self.support {
            FaceSupport::Plane(p) => Ok(p),
            FaceSupport::Surface(_) => Err("a spatial face is not a planar sweep profile".into()),
        }
    }
    pub fn on(&self) -> Option<EntRef> {
        match self.support { FaceSupport::Surface(s) => Some(s), FaceSupport::Plane(_) => None }
    }
    pub fn boundaries(&self) -> impl Iterator<Item = (&[EntRef], &[String])> {
        std::iter::once((self.edges.as_slice(), self.edge_names.as_slice()))
            .chain(self.holes.iter().map(|h| (h.edges.as_slice(), h.edge_names.as_slice())))
    }
}

/// A finite length in the drawing's user units.
#[derive(Clone, Debug)]
pub struct Length {
    value: f64,
    text: String,
}
impl Length {
    pub fn new(value: f64) -> Result<Self, String> {
        if !value.is_finite() {
            return Err("claim gap must be a finite length".into());
        }
        Ok(Self {
            value,
            text: String::new(),
        })
    }
    pub(crate) fn written(value: f64, text: String) -> Result<Self, String> {
        Ok(Self {
            text,
            ..Self::new(value)?
        })
    }
    pub fn value(&self) -> f64 {
        self.value
    }
    pub fn text(&self) -> &str {
        &self.text
    }
}

/// Complete argument forms. An inside claim cannot carry a gap.
#[derive(Clone, Debug)]
pub enum SolidRequirement {
    Inside,
    Fits { gap: Length },
    Clear { gap: Length },
}
impl SolidRequirement {
    pub fn word(&self) -> crate::constraints::SolidWord {
        use crate::constraints::SolidWord as W;
        match self {
            Self::Inside => W::Inside,
            Self::Fits { .. } => W::Fits,
            Self::Clear { .. } => W::Clear,
        }
    }
    pub fn gap(&self) -> Option<&Length> {
        match self {
            Self::Inside => None,
            Self::Fits { gap } | Self::Clear { gap } => Some(gap),
        }
    }
    /// Checked adapter for callers of the former word/gap API.
    pub(crate) fn from_word(word: crate::constraints::SolidWord, gap: f64) -> Result<Self, String> {
        use crate::constraints::SolidWord as W;
        match word {
            W::Inside if gap == 0.0 => Ok(Self::Inside),
            W::Inside => Err("inside takes no gap".into()),
            W::Fits => Ok(Self::Fits {
                gap: Length::new(gap)?,
            }),
            W::Clear => Ok(Self::Clear {
                gap: Length::new(gap)?,
            }),
        }
    }
}

/// A parsed, validated claim. Solid references and sweep dimensions are checked before creation.
///
/// ```compile_fail
/// fn change_sweep(sw: &mut gcs_core::model::Sweep) {
///     sw.from = f64::NAN;
/// }
/// ```
#[derive(Clone, Debug)]
pub struct SolidClaim {
    pub(crate) requirement: SolidRequirement,
    pub(crate) a: u32,
    pub(crate) b: u32,
    pub(crate) over: Option<Sweep>,
    pub(crate) stmt: u32,
}
impl SolidClaim {
    pub fn requirement(&self) -> &SolidRequirement {
        &self.requirement
    }
    pub fn solids(&self) -> (u32, u32) {
        (self.a, self.b)
    }
    pub fn over(&self) -> Option<&Sweep> {
        self.over.as_ref()
    }
    pub fn stmt(&self) -> u32 {
        self.stmt
    }
}

/// A cap used for placement, checked for surviving material after the drawing solves.
#[derive(Clone, Debug)]
pub struct SolidBearing {
    pub solid: u32,
    pub path: String,
    pub stmt: u32,
    pub span: crate::syntax::Span,
}

/// The interval a claim is swept over: a free variable of the drawing, or a named motion's roll
/// (in degrees), and where it runs.
#[derive(Clone, Debug)]
pub struct Sweep {
    pub(crate) name: String,
    pub(crate) from: f64,
    pub(crate) to: f64,
    pub(crate) dimension: crate::units::Dim,
    /// The motion whose roll is swept, where it is one: every solid placed under it is read at
    /// its `at:` advanced by the roll, and nothing is solved again.
    pub(crate) motion: Option<u32>,
}
impl Sweep {
    /// The motion whose roll the claim runs along, if it runs along one.
    pub fn motion(&self) -> Option<u32> {
        self.motion
    }
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn from(&self) -> f64 {
        self.from
    }
    pub fn to(&self) -> f64 {
        self.to
    }
    pub fn dimension(&self) -> crate::units::Dim {
        self.dimension
    }
    /// Convex interpolation avoids overflowing the difference of opposite extreme endpoints.
    pub fn sample(&self, index: usize, intervals: usize) -> Option<f64> {
        if intervals == 0 || index > intervals {
            return None;
        }
        if index == 0 {
            return Some(self.from);
        }
        if index == intervals {
            return Some(self.to);
        }
        let t = index as f64 / intervals as f64;
        let value = if self.from.signum() == self.to.signum() {
            self.from + (self.to - self.from) * t
        } else {
            self.from * (1.0 - t) + self.to * t
        };
        Some(value)
    }
}

/// A picture asked of a solid (§6.11): what of, cut where, drawn in which view.
#[derive(Clone, Debug)]
pub struct DerivedE {
    pub solid: u32,
    /// The plane it is drawn in; `None` is the page.
    pub plane: Option<u32>,
    /// A section's cutting plane.  `None` is a plain view.
    pub at: Option<u32>,
    /// A picture that is only its **dimensions** (§6.12): the sheet as a report.
    pub dims: bool,
    pub name: String,
    pub class: Classes,
}

/// A number a solid is swept or stood off by: the text a person wrote and what it came to.
///
/// It is settled by the flattener over the parameters in scope and is **never an unknown**, so
/// a solve moves it no more than it moves a spline's knots.  The text is kept because a drawing
/// reads better for saying `fw + D / 2` than `18`, which is the same reason a dimension carries
/// its expression.
#[derive(Clone, Debug, PartialEq)]
pub struct Extent {
    pub text: String,
    pub value: f64,
}

impl Extent {
    /// An extent nobody wrote: a caller building a solid in Rust, and what a gesture would
    /// splice.  The text is the number, printed as the source printer prints one.
    pub fn at(value: f64) -> Extent {
        Extent { text: crate::syntax::num(value), value }
    }
}

/// Which way a partial revolution turns — a word, never a sign (§9.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sense {
    Ccw,
    Cw,
}

/// **What a solid is** (Solvent §6.9).  Three forms and no fourth: a face swept two ways, and a
/// body over other solids.
#[derive(Clone, Debug)]
pub enum SolidDef {
    /// A rigid instance of another solid at one angle of a named motion family.
    Placed { source: u32, motion: u32, at: Extent },
    /// Continuous material swept under a named motion; angular extents are radians.
    Swept { source: u32, motion: u32, from: Extent, to: Extent },
    /// A face swept along its plane's normal, between two signed coordinates along it.  `from`
    /// and `to` are ordinates and their signs are arithmetic; `depth: d` is the draughtsman's
    /// spelling of `from: -d, to: 0`, the material behind the face the view shows.
    Prism { face: u32, from: Extent, to: Extent },
    /// Sections at the endpoints of a directed line or circular arc.
    Loft { face: u32, end: Option<u32>, guide: EntRef },
    /// A prism spanning the target's additive material, evaluated after solving.
    Through { face: u32, body: u32 },
    /// A face swept about a line **in its own plane**, through `sweep` (a full turn where the
    /// document writes none), right-handed about the line's own `p1 → p2` unless `sense: cw`.
    Revolve { face: u32, axis: u32, sweep: Extent, sense: Sense },
    /// **The body rule, and the whole of it**: its stock, plus everything in `union` with it,
    /// minus everything that `cut`s it, within everything that `bound`s it.  All three are sets, so
    /// the statements that fill them may be written anywhere in any order (P2) — which is what
    /// a feature tree, folding a *sequence*, is not.  Difference and intersection commute, so
    /// the last two need no order between them; union comes first.  A design needing the
    /// other order names the intermediate, and then there are two solids because there are
    /// two things.
    Body { stock: u32, on: Vec<u32>, through: Vec<u32>, bound: Vec<u32> },
}

/// A solid, as the document names it.
#[derive(Clone, Debug)]
pub struct SolidE {
    pub def: SolidDef,
    /// What the document calls it — see `FaceE::name`.
    pub name: String,
    pub class: Classes,
}

/// A generating-profile junction, or an envelope intersected by a boundary surface.
#[derive(Clone, Debug)]
pub struct SeamE {
    pub first: EntRef,
    pub second: EntRef,
    pub name: String,
    pub class: Classes,
}

/// A seam between two corner identities, sliced along a named spatial line.
#[derive(Clone, Debug)]
pub struct EdgeE {
    pub seam: u32,
    pub start: u32,
    pub end: u32,
    pub along: u32,
    pub name: String,
    pub class: Classes,
}

/// A spatial corner at the intersection of two seam identities.
#[derive(Clone, Debug)]
pub struct VertexE {
    pub first: u32,
    pub second: u32,
    pub name: String,
    pub class: Classes,
}

/// A source surface or envelope clipped by closed material regions.
#[derive(Clone, Debug)]
pub struct PatchE {
    pub source: EntRef,
    pub inside: Vec<u32>,
    pub outside: Vec<u32>,
    pub name: String,
    pub class: Classes,
}

/// A named implicit envelope, evaluated after solving. Roll bounds are radians.
#[derive(Clone, Debug)]
pub struct EnvelopeE {
    pub surface: u32,
    pub motion: u32,
    pub roll: [f64;2],
    pub name: String,
    pub class: Classes,
}

#[derive(Clone, Debug)]
pub struct SurfaceE {
    pub solid: u32,
    pub edge: EntRef,
    /// Angular limits in radians, measured along the source revolution's declared sense.
    /// None retains the entire source sweep. Parameters keep the original source chart.
    pub span: Option<[f64;2]>,
    pub name: String,
    pub class: Classes,
}

/// One shared angular parameter drives every kind. `advance` is a length (model
/// units) travelled along the axis per full turn of that parameter: zero makes a
/// rotation, nonzero a screw, and a translation moves without turning.
#[derive(Clone, Debug)]
pub enum MotionDef {
    Rotation { axis: u32, ratio: f64, phase: f64, advance: f64 },
    /// A turn in a view about a point of it (`motion(about: c, …)`): a rotation about the line
    /// through `centre` square to its view, which is what it reads as in space; in the plane its
    /// centre is a column of whatever is generated under it (`generate.rs`).
    Turn { centre: u32, ratio: f64, phase: f64 },
    Translation { axis: u32, advance: f64 },
    Relative { source: u32, observer: u32 },
}

#[derive(Clone, Debug)]
pub struct MotionE {
    pub def: MotionDef,
    pub name: String,
    pub class: Classes,
    /// The numbers written as measurements of the drawing (`ratio: length(a) / length(b)`),
    /// worked out whenever the motion is read and standing in for `def`'s own: see
    /// `MotionE::rotation`.  Empty for every motion whose numbers are plain.
    pub measured: Vec<crate::measure::MotionMeasure>,
}

impl MotionE {
    /// A rotation's `(ratio, phase, advance)` — phase in radians — on the drawing as it now
    /// stands: `def`'s numbers, with each one written as a measurement worked out afresh.  The
    /// one reader of a rotation's numbers, so a snapshot and a cache key cannot disagree.
    pub fn rotation(&self, sk: &crate::model::Sketch) -> Result<(f64, f64, f64), String> {
        let (ratio, phase, advance) = match self.def {
            MotionDef::Rotation { ratio, phase, advance, .. } => (ratio, phase, advance),
            MotionDef::Turn { ratio, phase, .. } => (ratio, phase, 0.0),
            _ => return Err(format!("`{}` is not a rotation", self.name)),
        };
        let mut out = (ratio, phase, advance);
        for m in &self.measured {
            let v = m.value.value(sk).map_err(|e| format!("`{}`: {e}", self.name))?;
            match m.slot {
                crate::measure::MotionSlot::Ratio => out.0 = v,
                crate::measure::MotionSlot::Phase => out.1 = v.to_radians(),
                crate::measure::MotionSlot::Advance => out.2 = v,
            }
        }
        Ok(out)
    }

    /// A translation's advance, likewise.
    pub fn advance(&self, sk: &crate::model::Sketch) -> Result<f64, String> {
        let MotionDef::Translation { advance, .. } = self.def else {
            return Err(format!("`{}` is not a translation", self.name));
        };
        match self.measured.iter().find(|m| m.slot == crate::measure::MotionSlot::Advance) {
            Some(m) => m.value.value(sk).map_err(|e| format!("`{}`: {e}", self.name)),
            None => Ok(advance),
        }
    }
}

impl SolidE {
    /// Boolean operands only; a through-extent target is a separate evaluation dependency.
    pub fn operands(&self) -> Vec<u32> {
        match &self.def {
            SolidDef::Placed { source, .. } | SolidDef::Swept { source, .. } => vec![*source],
            SolidDef::Prism { .. } | SolidDef::Revolve { .. } | SolidDef::Through { .. } | SolidDef::Loft { .. } => Vec::new(),
            SolidDef::Body { stock, on, through, bound } => {
                let mut v = vec![*stock];
                v.extend(on.iter().copied());
                v.extend(through.iter().copied());
                v.extend(bound.iter().copied());
                v
            }
        }
    }

    /// The face it is swept from, if it is swept from one.
    pub fn face(&self) -> Option<u32> {
        match &self.def {
            SolidDef::Prism { face, .. } | SolidDef::Revolve { face, .. } | SolidDef::Through { face, .. } | SolidDef::Loft { face, .. } => Some(*face),
            SolidDef::Body { .. } | SolidDef::Placed { .. } | SolidDef::Swept { .. } => None,
        }
    }
}

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
            support: FaceSupport::Plane(plane.map(|p| p as u32)),
            name: name.to_string(),
            class: Classes::default(),
        });
        self.faces.len() - 1
    }

    /// A solid: a face swept, or a term over other solids.  It allocates no parameter, which is
    /// the whole of what "nothing three-dimensional is ever solved for" comes to in code.
    pub fn solid(&mut self, def: SolidDef, name: &str) -> usize {
        self.solids.push(SolidE { def, name: name.to_string(), class: Classes::default() });
        self.solids.len() - 1
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
        // A swept solid's field mesh does not depend on the view's pixel length: one mesh
        // serves every zoom, rather than a refinement per wheel tick.
        let policy = if crate::solid::has_sweep(self, i) { crate::solid::ApproximationPolicy::Mesh } else { policy };
        let key = crate::solid::reads(self, i, 0.0);
        let slot = (i, policy.cache_key());
        if let Some((old, value)) = self.solid_cache.borrow().get(&slot) {
            if *old == key { return value.clone(); }
        }
        let value = crate::solid::EvaluatedSolid::evaluate(self, i, policy).map(std::rc::Rc::new);
        let mut cache = self.solid_cache.borrow_mut();
        cache.retain(|(index, _), (old, _)| *index != i || *old == key);
        // Bound zoom-history memory without evicting report and mesh entries.
        if cache.keys().filter(|(index, (kind, _))| *index == i && *kind == 2).count() >= 8 {
            cache.retain(|(index, (kind, _)), _| *index != i || *kind != 2);
        }
        cache.insert(slot, (key, value.clone()));
        value
    }

    /// Solid `i`'s exact B-rep (`solid::Exact`), or why this kernel does not build it: built once
    /// for the geometry it reads, whatever approximation asks.
    pub fn exact_solid(&self, i: usize) -> Result<std::rc::Rc<crate::solid::Exact>, String> {
        let key = crate::solid::reads(self, i, 0.0);
        if let Some((old, value)) = self.exact_cache.borrow().get(&i) {
            if *old == key { return value.clone(); }
        }
        let value = crate::solid::build_exact(self, i).map(std::rc::Rc::new);
        self.exact_cache.borrow_mut().insert(i, (key, value.clone()));
        value
    }

    /// Give swept solid `i` its exact B-rep, built elsewhere (`brep::export::supply_exact`) against
    /// the drawing as it stands now: its evaluation then reads the B-rep, as a static solid's does
    /// (`EvaluatedSolid::from_brep`), until the drawing changes.
    pub fn supply_exact_solid(&self, i: usize, exact: crate::solid::Exact) {
        let key = crate::solid::reads(self, i, 0.0);
        self.exact_cache.borrow_mut().insert(i, (key, Ok(std::rc::Rc::new(exact))));
    }

    /// The exact B-rep supplied for swept solid `i` against the drawing as it stands, if any.
    pub fn supplied_exact_solid(&self, i: usize) -> Option<std::rc::Rc<crate::solid::Exact>> {
        let key = crate::solid::reads(self, i, 0.0);
        match self.exact_cache.borrow().get(&i) { Some((old, Ok(x))) if *old == key => Some(x.clone()), _ => None }
    }

    /// **What a host meshing swept solids elsewhere has to mesh** (`FieldMeshing::Deferred`): the
    /// objects (`overview::objects`) with a continuous sweep among their operands, each with the
    /// key of the drawing it is a surface of (`FieldJob`). A swept solid that is no object is left
    /// out: nothing shows or exports it, so its surface is never asked for.
    pub fn field_jobs(&self) -> Vec<crate::solid::FieldJob> {
        crate::overview::objects(self).into_iter().filter(|&i| self.is_swept(i))
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
        self.solid_cache.borrow_mut().retain(|(index, _), _| *index != i);
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
