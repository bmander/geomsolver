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
mod solids;
mod region;
mod solid_eval;
mod construction;
mod parameters;
mod geometry;
mod measure;
mod topology;
mod attitude;
mod turns;
mod duals;
mod rates;
mod caches;
mod running;

pub use entities::{
    Param, EntKind, Ends, Run, Field, EntRef, PointE, LineE, CircleE, AxisE, ArcE, SplineE,
    FrameE, Chart, Joint,
    LiftE, article,
};
pub use curves::{CURVE_STEPS, CurveDef, CurveBody, CurveE, Extremal, Home, Trim, extremal_def, whole};
pub use solids::{
    FaceLoop, FaceE, Length, SolidRequirement, SolidClaim, SolidBearing, Sweep,
    DerivedE, Extent, Roll, Sense, SolidDef, SolidE, EnvelopeE,
    Ease, Flank, Held, MeshingE, SetKind, SetShape, SurfaceE, TracedE,
    MotionDef, MotionE, station_prefix, FamilyLeg, Leg,
};
pub use region::{RegionShape, RegionTerm, TermNumber, FaceRef, Target};
pub use solid_eval::{Way, turn_of};
pub use construction::{ThreePointArc, three_point_arc};
pub use turns::{Turn, Derivation};
pub use duals::{Dual, Move, Moves, Toward};
pub use rates::{RateRow, RateWord};
pub use caches::Caches;
pub use running::Running;
pub use geometry::{Box2, grow};
pub use topology::{edge_ends, expand};
pub(crate) use measure::polyline_distance;
pub use measure::{
    signed_point_to_line, point_to_drawn, seg_distance, pick, pick_by, angle_between, on_radius,
    distance_between, orientation, orientation_xy, increments,
};

/// An unknown the source declared (`Sketch::declared`): the dimension it was declared with
/// and, where one was written, the number its solve begins at, in user units (degrees).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Declared {
    pub dim: crate::units::Dim,
    pub seed: Option<f64>,
}

/// A place along one curve that several contacts own (`t == s`): the unknown, and the curve it
/// runs along, which every owner must stand on (`constraints::validate`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SharedPlace {
    pub param: u32,
    pub along: EntRef,
}

/// A face family's running number (`Sketch::held`): its fixed parameter, in the units a person
/// reads (degrees for an angle), and what it is — or a number of the geometry a station reads
/// (`reads`, the entity and which of its scalars: `HELD_X`, `HELD_Y`, `HELD_Z`, `HELD_R`), held
/// at what that geometry stands at whenever the expressions are worked out.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeldNumber {
    pub param: u32,
    pub dim: crate::units::Dim,
    pub reads: Option<(EntRef, u8)>,
}

/// Which scalar of the geometry a held number reads (`HeldNumber::reads`): a point's coordinates
/// on its page (or in space), a circle's or an arc's radius.
pub const HELD_X: u8 = 0;
pub const HELD_Y: u8 = 1;
pub const HELD_Z: u8 = 2;
pub const HELD_R: u8 = 3;

#[derive(Default, Clone, Debug)]
pub struct Sketch {
    pub roles: BTreeMap<EntRef, crate::semantics::GeometryRoles>,
    pub params: Vec<Param>,
    pub points: Vec<PointE>,
    pub lines: Vec<LineE>,
    pub circles: Vec<CircleE>,
    /// The axes: directed lines in space, drawn in no view.
    pub axes: Vec<AxisE>,
    /// A fixed Param holding 0 that an axis's direction is read against (`Sketch::origin_param`).
    pub zero: Option<u32>,
    pub arcs: Vec<ArcE>,
    pub splines: Vec<SplineE>,
    pub frames: Vec<FrameE>,
    /// The hidden points in space a spatial relation reads — see `LiftE`.  Derived state,
    /// re-minted rather than saved.
    pub lifts: Vec<LiftE>,
    /// Each tangency's derivative (§6.21): the point it moves, what moves it and the geometry the
    /// use made — see `Dual`.  Document state, saved and grafted; its tangent columns are minted
    /// by the rows that read them.
    pub duals: Vec<Dual>,
    /// The points drawn in further planes (`point in P, G`, §6.7), each with its **twins**: a
    /// point of its own in each further plane, tied to it in space (`Sketch::tie_twin`,
    /// `hold_twins`).  A reader drawn in a plane reads the point there by its twin
    /// (`Sketch::twin_in`).
    pub twins: BTreeMap<usize, Vec<usize>>,
    /// **What rides on frames** (#383): a point drawn in nested `in` blocks, or an envelope or
    /// a swept solid riding on its observer, to the frames of its path, outermost first — its
    /// carriers are their joints (`Sketch::carriers`).  At rest it is drawn where it stands: the
    /// path enters no row but a reading at a roll.
    pub rides: BTreeMap<EntRef, Vec<u32>>,
    pub curves: Vec<CurveE>,
    /// The faces and solids the document names (§6.8, §6.9).  Built after every other kind,
    /// since a face is written over edges and a solid over faces and solids; evaluated after
    /// the drawing is solved, since nothing about either is an unknown.
    pub faces: Vec<FaceE>,
    pub solids: Vec<SolidE>,
    pub surfaces: Vec<SurfaceE>,
    pub motions: Vec<MotionE>,
    pub envelopes: Vec<EnvelopeE>,
    /// The meshings of pairs of flanks (#283): tooth contact analyses.
    pub meshings: Vec<MeshingE>,
    /// The bodies traced in space (#326): curves and surfaces a contact holds to.
    pub traced: Vec<TracedE>,
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
    /// What the document calls each plane, for the one diagnostic that has to say so — see
    /// `plane_name`.
    pub plane_names: BTreeMap<u32, String>,
    /// The curve families this document defines.  Document state like `branches`: a curve
    /// instance names one by index.
    pub curve_defs: Vec<CurveDef>,
    /// A `ring`'s copies, each the representative turned (`model/turns.rs`): document state like
    /// `branches`, saved and grafted, from which the derived parameters are worked out.
    pub turns: Vec<Turn>,
    pub constraints: Vec<Constraint>,
    /// Recorded root choices (Stage 5), persisted with the document.
    pub branches: BTreeMap<String, i32>,
    /// Where a dimension's callout has been dragged to, by constraint id, in the frame that
    /// callout hangs off — see `callout::Frame`.  Document state, like `branches`: an entry
    /// only exists for a dimension somebody has moved, and dropping one puts that callout back
    /// where the layout would have placed it.
    pub placements: BTreeMap<u32, (f64, f64)>,
    /// The unknowns the document's dimension expressions read, by name, each an index into
    /// `params` — see `expr::Free`.  Derived state, owned by `expr::evaluate`: it allocates one
    /// the first time a dimension reads the name (in a Solvent document, a declared unknown) and
    /// retires it when the last reader stops reading it.
    pub free_vars: BTreeMap<String, u32>,
    /// The contact parameters several contacts own together, by the name they are pinned to
    /// (`path tangent(t == s) ground`) — allocated by `Sketch::add` for the first contact naming
    /// it and handed to the rest.  The constraints hold the index (`Arg::Param`), so this table
    /// is what a rebuild (`io::graft`) and the document writer read the name back from.
    pub shared: BTreeMap<String, SharedPlace>,
    /// **The face families' running numbers** (#243 L1), by name (`#f12.s`): each a *fixed*
    /// parameter, the station the family's instance is solved at.  `expr::evaluate` reads one
    /// as a number, whatever the text does with it, so it is no free variable; a contact pinned
    /// to one (`t == s`) owns its parameter (`Sketch::add`).  The drawn station stands at the
    /// middle of the interval; `solid::family` moves it.
    pub held: BTreeMap<String, HeldNumber>,
    /// **The rate rows** (#339 D3, `rates.rs`): what a station states of its rates along its
    /// family's running number.  No row of the drawing — the drawn station is the home, its
    /// initial value — but what the family integrates the station by (`solid::rated`).  Grafted
    /// with the entities they name.
    pub rates: Vec<RateRow>,
    /// **The parameters a chain's later leg carries** (#363 S4): a station that states a rate,
    /// drawn on a link of a chain past the first, has no home there — its frame is the one the
    /// last leg's flow carried across the joint, which nothing in the drawing knows — so its rows
    /// are `Constraint::carried` and its own parameters held where they were seeded, these, until
    /// its family's part frees them (`solid::family::part_of`).  Set by the elaborator
    /// (`program::solids::carry_legs`).
    pub carried: std::collections::BTreeSet<u32>,
    /// **The splines drawn over points in space** (#363 S3), each to the curve traced in space it
    /// is (`traced::RowKind::Spline`): its name reads the traced curve, and the spline itself,
    /// kept so no index moves, is no primitive — neither drawn nor grafted.  Set by the
    /// elaborator (`program::entities::splines_in_space`).
    pub spline_traced: std::collections::BTreeMap<u32, u32>,
    /// **The points an arc over points in space is drawn over** (#363 S6), each to the point in
    /// space it stands for: the arc's own centre, start and end in the plane it minted, tied to
    /// the points written (`program::entities::arcs_in_space`), so a chain's links still meet
    /// where they were threaded.
    pub stands_for: std::collections::BTreeMap<u32, u32>,
    /// **The axes a circle about a point in space is gauged by** (#363 S6), each to the `u` and
    /// `v` axes of the plane it minted: the plane's turn about the circle's normal is held square
    /// to it, and it is taken again where a solve ends (`retake_round_gauges`), so a normal that
    /// swings onto it never leaves that turn unheld at a solved pose.  Set by the elaborator
    /// (`program::entities::circles_in_space`).
    pub round_gauges: std::collections::BTreeMap<u32, [u32; 2]>,
    /// **The arcs a chain walks from their end** (#363 S4), each drawn from there: its start and
    /// end swapped and its turn reversed, the same arc, so the chain enters it at its start and a
    /// family runs it forward.  A name still reads the points as written: `k.start` is the end it
    /// is drawn to (`program::resolve::follow`).  Set by the elaborator
    /// (`program::entities::walked_back`, read by `arcs_in_space`).
    pub walked_back: std::collections::BTreeSet<u32>,
    /// Each free curve's energy as `settle_variational` last read it: its statements, pegs and
    /// whether its length is free.  Derived state, never saved.
    pub variational: Vec<crate::variational::Energy>,
    /// Physical dimensions inferred by expression evaluation, in user units (angles in degrees).
    pub free_dimensions: BTreeMap<String, crate::units::Dim>,
    /// The unknowns the source declared, by name — an input nothing binds (`param beta:
    /// Angle hint(30deg)`), a formal left unbound with a seed — and what each is: read by
    /// `expr::evaluate` when it allocates the unknown (its seed, its dimension).  Set by
    /// elaboration and never written to a document.
    pub declared: BTreeMap<String, Declared>,
    /// **The parameter axes the source declares** (#391 S1), by name: `param t: Angle over
    /// (0deg, 360deg)`, a number that runs over an interval rather than one the solve answers
    /// for.  Represented once, as every running number is (`Running`); nothing reads one yet.
    /// Set by elaboration and never written to a document, as `declared` is.
    pub parameters: BTreeMap<String, Running>,
    /// **Where a declared parameter's march stands the drawing** (#398 L): the instant shown, and
    /// the drawing at home, where it was drawn, to go back to (`follow::stand`, `follow::home`).
    /// Presentation, as a wobble is: what is drawn moves and the document does not — a march
    /// starts from home, and no seed is written back from an instant (`edit::commit_seeds`).
    /// Never written to a document.
    pub standing: Option<crate::follow::Standing>,
    /// What the sketch remembers and could work out again — polylines, evaluated and exact
    /// solids, regions' readings, bodies' faces — and what a clone keeps of it (`Caches`).
    pub caches: Caches,
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
    /// Bumped whenever a solid's shape is changed in place — an extrusion being sized
    /// (`set_prism`) — which neither re-elaborates nor moves a parameter, so a binding caching a
    /// mesh keys on it too.
    pub shape_epoch: u32,
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
            EntKind::Axis => self.axes[e.i()].class.clone(),
            EntKind::Arc => self.arcs[e.i()].class.clone(),
            EntKind::Spline => self.splines[e.i()].class.clone(),
            EntKind::Frame => self.frames[e.i()].class.clone(),
            EntKind::Curve => self.curves[e.i()].class.clone(),
            EntKind::Face => self.faces[e.i()].class.clone(),
            EntKind::Solid => self.solids[e.i()].class.clone(),
            EntKind::Surface => self.surfaces[e.i()].class.clone(),
            EntKind::Motion => self.motions[e.i()].class.clone(),
            EntKind::Envelope => self.envelopes[e.i()].class.clone(),
            EntKind::Meshing => self.meshings[e.i()].class.clone(),
            EntKind::Traced => self.traced[e.i()].class.clone(),
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
            EntKind::Axis => self.axes.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Arc => self.arcs.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Spline => self.splines.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Frame => self.frames.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Curve => self.curves.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Face => self.faces.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Solid => self.solids.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Surface => self.surfaces.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Motion => self.motions.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Envelope => self.envelopes.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Meshing => self.meshings.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Traced => self.traced.get_mut(e.i()).map(|x| &mut x.class),
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
