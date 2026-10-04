//! Parameters, primitives and the Sketch container.
//!
//! Every scalar degree of freedom is a `Param`.  Primitives are thin bundles of Param indices;
//! the Sketch owns the ordered list of Params (its parameter vector) and the ordered list of
//! Constraints.  Ordering is deterministic by construction — insertion order, never hashing — so
//! identical edits give bit-identical solves.
//!
//! Identity is an integer everywhere: a Param is its index, an entity is `(kind, index)`, and a
//! constraint is a monotonic `id`.  The bindings intern their proxies on those, so `is` / `===`
//! keep working across the FFI without any pointer ever leaving the core.

use crate::constraints::Constraint;
use crate::style::Classes;
use std::collections::BTreeMap;

// Private responsibility modules; existing model paths remain the public API.
mod entities;
mod curves;
mod spatial;
mod construction;
mod parameters;
mod geometry;
mod measure;
mod topology;
mod attitude;

pub use entities::{
    Param, EntKind, Field, EntRef, PointE, LineE, CircleE, SphereE, AxialE, ArcE, SplineE, FrameE,
    PlaneE, Att,
    LiftE,
};
pub use curves::{CURVE_STEPS, CurveDef, CurveBody, CurveE, Home, Trim, whole};
pub use spatial::{
    FaceLoop, FaceSupport, FaceE, Length, SolidRequirement, SolidClaim, SolidBearing, Sweep,
    DerivedE, Extent, Sense, SolidDef, SolidE, SeamE, EdgeE, VertexE, PatchE, EnvelopeE, SurfaceE,
    MotionDef, MotionE,
};
pub use construction::{ThreePointArc, three_point_arc};
pub use geometry::{Box2, grow};
pub use topology::{edge_ends, expand};
pub(crate) use measure::polyline_distance;
pub use measure::{
    signed_point_to_line, point_to_drawn, seg_distance, pick, pick_by, angle_between, on_radius,
    distance_between, orientation, orientation_xy, increments,
};

/// A place along one curve that several contacts own (`t == s`): the unknown, and the curve it
/// runs along, which every owner must stand on (`constraints::validate`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SharedPlace {
    pub param: u32,
    pub along: EntRef,
}

#[derive(Default, Clone, Debug)]
pub struct Sketch {
    pub roles: BTreeMap<EntRef, crate::semantics::GeometryRoles>,
    pub params: Vec<Param>,
    pub points: Vec<PointE>,
    pub lines: Vec<LineE>,
    pub circles: Vec<CircleE>,
    /// The spheres, appended last of the drawn kinds; empty in a document that declares none.
    pub spheres: Vec<SphereE>,
    /// The cones and cylinders, after the spheres; likewise.
    pub cones: Vec<AxialE>,
    pub cylinders: Vec<AxialE>,
    pub arcs: Vec<ArcE>,
    pub splines: Vec<SplineE>,
    pub planes: Vec<PlaneE>,
    /// The hidden points in space a spatial relation reads — see `LiftE`.  Derived state,
    /// re-minted rather than saved.
    pub lifts: Vec<LiftE>,
    pub curves: Vec<CurveE>,
    /// The faces and solids the document names (§6.8, §6.9).  Built after every other kind,
    /// since a face is written over edges and a solid over faces and solids; evaluated after
    /// the drawing is solved, since nothing about either is an unknown.
    pub faces: Vec<FaceE>,
    pub solids: Vec<SolidE>,
    pub surfaces: Vec<SurfaceE>,
    pub motions: Vec<MotionE>,
    pub envelopes: Vec<EnvelopeE>,
    pub patches: Vec<PatchE>,
    pub seams: Vec<SeamE>,
    pub vertices: Vec<VertexE>,
    pub edges: Vec<EdgeE>,
    /// The pictures the document asks for (§6.11): `view(body) in right`, `section(body, at: mid)
    /// in front`.  Not entities — nothing on the sheet is held to one, and no solve moves one —
    /// but document state like `branches`, saved and grafted with everything else.
    pub derived: Vec<DerivedE>,
    /// The claims the document makes about its solids (§9.8).  Judged at the end of a diagnosis
    /// and never solved: a solid claim compiles no row, so it cannot move geometry, weld two
    /// figures into one drag part or paint a sketch Over — the bargain §9.7 already struck for a
    /// 2D claim, one stratum further out.
    pub solid_claims: Vec<SolidClaim>,
    pub solid_bearings: Vec<SolidBearing>,
    /// The datum points the **page-placement gauge** holds: a solved view's origin and
    /// toward, where no statement of the document names them.  Their params are `fixed`, so no
    /// solve moves them and no ledger counts them; this set is what says the hold is the gauge's
    /// and not a `fix`'s, so a writeback into the source never spells one.  (A lifted program
    /// does fix them: it states every view, and over a stated view the gauge holds nothing.)
    /// Derived state, set at elaboration and never written to a document.
    pub page_held: std::collections::BTreeSet<u32>,
    /// The planes a **mate** places (§6.10): written `from: P` with neither `fold:` nor
    /// `offset:`, they say which plane they are parallel to and leave where they stand to one
    /// `against`.  Recorded here so the placement walk knows which offsets are its to write and
    /// which the document already fixed — a plane that says `offset: 12mm` is not placed twice.
    pub placed_planes: std::collections::BTreeSet<u32>,
    /// What the document calls each plane, for the one diagnostic that has to say so — see
    /// `plane_name`.
    pub plane_names: BTreeMap<u32, String>,
    /// The curve families this document defines.  Document state like `branches`: a curve
    /// instance names one by index.
    pub curve_defs: Vec<CurveDef>,
    pub constraints: Vec<Constraint>,
    /// Recorded root choices (Stage 5), persisted with the document.
    pub branches: BTreeMap<String, i32>,
    /// Where a dimension's callout has been dragged to, by constraint id, in the frame that
    /// callout hangs off — see `callout::Frame`.  Document state, like `branches`: an entry
    /// only exists for a dimension somebody has moved, and dropping one puts that callout back
    /// where the layout would have placed it.
    pub placements: BTreeMap<u32, (f64, f64)>,
    /// The free variables the document's dimension expressions read, by name, each an index
    /// into `params` — see `expr::Free`.  Derived state, owned by `expr::evaluate`: it allocates
    /// one the first time a name nothing defines is read and retires it when the last reader
    /// stops reading it, so nothing else in the document has to know they exist.
    pub free_vars: BTreeMap<String, u32>,
    /// The contact parameters several contacts own together, by the name they are pinned to
    /// (`path tangent(t == s) ground`) — allocated by `Sketch::add` for the first contact naming
    /// it and handed to the rest.  The constraints hold the index (`Arg::Param`), so this table
    /// is what a rebuild (`io::graft`) and the document writer read the name back from.
    pub shared: BTreeMap<String, SharedPlace>,
    /// Physical dimensions inferred by expression evaluation, in user units (angles in degrees).
    pub free_dimensions: BTreeMap<String, crate::units::Dim>,
    /// Each curve's polyline, remembered against everything it was computed from
    /// (`curve_polyline`).  A pick walks every drawn curve on every pointer move, and a traced
    /// curve's polyline is a march of `CURVE_STEPS` block solves — nine milliseconds a move on
    /// the gear, for a drawing that had not changed.  Interior, so `&self` readers share it; a
    /// cache and not state, since a miss recomputes what a hit remembers.
    pub polyline_cache: std::cell::RefCell<BTreeMap<usize, (Vec<f64>, Vec<(f64, f64)>)>>,
    /// Each solid's evaluated boundary, remembered against everything it was computed from
    /// (`solid::reads`).  `polyline_cache`'s bargain, for the same reason: a repaint asks every
    /// derived view for its edges, and a boundary is a sweep of classifications over a drawing
    /// that has not changed.
    pub solid_cache: std::cell::RefCell<BTreeMap<(usize, (u8, u64)), (Vec<f64>, Result<std::rc::Rc<crate::solid::EvaluatedSolid>, String>)>>,
    /// Each static solid's exact B-rep (`solid::Exact`), remembered against `solid::reads` alone:
    /// it does not depend on an approximation, so it is built once for a geometry and only meshed
    /// again for each one the evaluated solids above ask for.
    pub exact_cache:
        std::cell::RefCell<BTreeMap<usize, (Vec<f64>, Result<std::rc::Rc<crate::solid::Exact>, String>)>>,
    /// Swept solids' surfaces a host meshed elsewhere (`supply_field`), each against the
    /// `solid::reads` it was supplied under, so a moved drawing reads none.
    pub field_surfaces: std::cell::RefCell<BTreeMap<usize, (Vec<f64>, std::rc::Rc<crate::solid::FieldSurface>)>>,
    /// Where a swept solid with no supplied surface is meshed (`solid::FieldMeshing`): here, or —
    /// set by a host that meshes swept solids itself, a page with a worker — elsewhere, the solid
    /// refused until its surface is supplied rather than meshed on the thread that draws.
    pub field_meshing: std::cell::Cell<crate::solid::FieldMeshing>,
    /// The document's style sheet: what each class looks like (`style.rs`).  Presentation, and
    /// nothing the core computes reads it — it is here because it is document state, saved and
    /// grafted with everything else, and because the core resolving it is what keeps two front
    /// ends from disagreeing about how one drawing is drawn.
    pub sheet: crate::style::Sheet,
    /// What the document's numbers are in (`units.rs`).  Storing it costs the solve nothing —
    /// every kernel is homogeneous in length, so scaling a whole sketch moves no residual, no
    /// tolerance and no rank — but it is what makes `80mm` mean something and what lets a paste
    /// out of a document in inches arrive in a document in millimetres as the same drawing.
    pub units: crate::units::Units,
    /// Bumped whenever the sheet or a class changes, so a binding may cache the resolved styles
    /// against it.  Geometry moves every frame and presentation almost never does; the counter
    /// is what lets the second be read at the second's rate.
    pub style_epoch: u32,
    next_cid: u32,
}

impl Sketch {
    pub fn new() -> Sketch {
        Sketch::default()
    }

    // -- presentation (`style.rs`) ------------------------------------------

    /// Semantic intent of this entity, independent of its drawing style.
    pub fn roles_of(&self, e: EntRef) -> crate::semantics::GeometryRoles {
        self.roles.get(&e).copied().unwrap_or_default()
    }

    /// Fixed semantic selectors followed by authored presentation classes.
    pub fn class_of(&self, e: EntRef) -> Classes {
        let authored = match e.kind {
            EntKind::Point => Classes::default(),
            EntKind::Line => self.lines[e.i()].class.clone(),
            EntKind::Circle => self.circles[e.i()].class.clone(),
            EntKind::Sphere => self.spheres[e.i()].class.clone(),
            EntKind::Cone => self.cones[e.i()].class.clone(),
            EntKind::Cylinder => self.cylinders[e.i()].class.clone(),
            EntKind::Arc => self.arcs[e.i()].class.clone(),
            EntKind::Spline => self.splines[e.i()].class.clone(),
            EntKind::Plane => self.planes[e.i()].frame.class.clone(),
            EntKind::Curve => self.curves[e.i()].class.clone(),
            EntKind::Face => self.faces[e.i()].class.clone(),
            EntKind::Solid => self.solids[e.i()].class.clone(),
            EntKind::Surface => self.surfaces[e.i()].class.clone(),
            EntKind::Motion => self.motions[e.i()].class.clone(),
            EntKind::Envelope => self.envelopes[e.i()].class.clone(),
            EntKind::Patch => self.patches[e.i()].class.clone(),
            EntKind::Seam => self.seams[e.i()].class.clone(),
            EntKind::Vertex => self.vertices[e.i()].class.clone(),
            EntKind::Edge => self.edges[e.i()].class.clone(),
        };
        Classes(self.roles_of(e).selectors().map(str::to_string).chain(authored.0).collect())
    }

    /// Give an entity a class, or take one away.  The one write path, so the epoch a binding
    /// caches against cannot be missed.
    pub fn set_class(&mut self, e: EntRef, name: &str, on: bool) {
        let slot = match e.kind {
            EntKind::Point => return,
            EntKind::Line => self.lines.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Circle => self.circles.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Sphere => self.spheres.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Cone => self.cones.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Cylinder => self.cylinders.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Arc => self.arcs.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Spline => self.splines.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Plane => self.planes.get_mut(e.i()).map(|x| &mut x.frame.class),
            EntKind::Curve => self.curves.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Face => self.faces.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Solid => self.solids.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Surface => self.surfaces.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Motion => self.motions.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Envelope => self.envelopes.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Patch => self.patches.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Seam => self.seams.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Vertex => self.vertices.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Edge => self.edges.get_mut(e.i()).map(|x| &mut x.class),
        };
        if let Some(c) = slot {
            c.set(name, on);
            self.style_epoch = self.style_epoch.wrapping_add(1);
        }
    }

    /// The whole sheet, replaced.  Elaboration's write path, and the other half of the epoch.
    pub fn set_sheet(&mut self, sheet: crate::style::Sheet) {
        self.sheet = sheet;
        self.style_epoch = self.style_epoch.wrapping_add(1);
    }

    /// What an entity is drawn with: the base sheet under the document's, cascaded over its
    /// classes.  **The core resolves; a front end strokes what it is handed** — the same seam
    /// `callout.rs` and `curve::tessellate` sit on, so every front end draws one drawing alike.
    pub fn style_of(&self, e: EntRef) -> crate::style::Style {
        // a kind may carry a class it was never given — a plane's datum glyph is `.plane` —
        // under whatever the declaration says, so the document's rule still wins
        let mut classes = self.class_of(e);
        if let Some(c) = e.kind.implicit_class() {
            classes.0.insert(0, c.to_string());
        }
        crate::style::resolve(&self.sheet, &classes)
    }

    /// What a *named* class list comes to, spelled as a declaration spells one: `"dimension"`,
    /// or `"dimension reference"`.  The drawing's chrome asks this — a dimension callout is not
    /// an entity and carries no class of its own, but its ink is shared by every callout in the
    /// document, which is exactly what a class is for.
    ///
    /// A list rather than one name because a claimed dimension *is* a dimension: it takes the
    /// shared rule and then the one that says how it differs, which is how a caller gets a
    /// document's `style .dimension` on a reference dimension too.
    pub fn style_named(&self, classes: &str) -> crate::style::Style {
        let list = Classes(classes.split_whitespace().map(str::to_string).collect());
        crate::style::resolve(&self.sheet, &list)
    }

    /// What the document calls a plane.  A plane carries no name of its own — the source map
    /// holds those — so this is the label a *diagnostic* uses, and it is the one thing here that
    /// would rather have had one.
    pub fn plane_name(&self, i: usize) -> String {
        self.plane_names.get(&(i as u32)).cloned().unwrap_or_else(|| format!("v{i}"))
    }
}
