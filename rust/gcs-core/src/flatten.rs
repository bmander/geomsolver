//! Instance expansion: a program with components and repetition becomes a flat list of statements.
//!
//! This is spec §14.1's first phase, and it is where the language's structure *goes*.  Everything
//! after it — building the geometry, adding the constraints, evaluating the expressions — works on
//! a list with no components and no blocks in it, exactly as it did before either existed.
//!
//! **Connection is aliasing, and aliasing is free.**  Passing an entity to an instance does not
//! add a constraint; it makes two names denote one entity, which costs no residual and cannot
//! be violated (spec P1).  That is not an optimisation — it is why a
//! component boundary is free, and it needs nothing from the model at all: a name resolves to an
//! entity, and several names may resolve to the same one.
//!
//! Names go out of the top absolute — `t#3.lead` — and are never printed.  A program is printed
//! by lifting the *sketch* it elaborated to (`program::to_program`), which mints `p0`, `l1` and
//! the rest, so an internal name has no spelling to keep valid.

use crate::expr::{self, Aff};
use crate::ir::PathStep;
use crate::program::{Code, Diag};
use crate::syntax::{
    build_rank, under_root, Block, BlockKind, Component, CurveTarget, Decl, Kid, Name, OpenJoint,
    OpenNamed, OpenSide, Program, Ref, Seg, Span, Stmt, StmtKind, Ty,
};
use crate::units::Units;
use std::collections::{BTreeMap, BTreeSet};

// Expansion state stays here; each private module owns one phase of the walk.
mod values;
mod expand;
mod bindings;
mod resolve;
mod words;
mod apply;
mod sets;
mod faces;
mod interfaces;
mod keys;
mod steps;
mod through;
mod image;
mod carried;
mod valued;
mod same;
mod table;
mod framed;
mod bound;
mod members;
mod lists;

pub use faces::{FaceAxis, FacePlane};
pub use bound::Bound;

pub(crate) use words::{faults as word_faults, file_number_names};
pub(crate) use interfaces::check as interface_faults;
pub(crate) use keys::{frame_datum, frame_datums, frame_instance, image, image_datum, instance_frame,
                      is_probe};
use crate::lowering::Lowered;

pub(crate) use values::{substitute_with, value_aff, value_read};
use values::{free, typed, value_of, reads_geometry};
use resolve::Lookup;
use table::{Table, Vals};
use crate::syntax::ref_text;

/// No set walked as a set for want of an element: `SetState::unlowered` where nothing was.
static NONE: BTreeSet<String> = BTreeSet::new();

/// How deep components and blocks may nest.  A document is untrusted input and
/// `wasm32-unknown-unknown` aborts rather than unwinding, so recursion is bounded here.
pub const MAX_DEPTH: usize = 32;

/// How many statements an expansion may produce.  `repeat 1000000` is a program somebody can
/// write; running out of memory is not the answer it should get.
pub const MAX_FLAT: usize = 200_000;

/// What a `next` or a `prev` means where a statement stands.
#[derive(Clone)]
struct Cyc {
    prefix: String,
    k: usize,
    n: usize,
}

/// The `ring` a statement stands in (§12.3): the prefix every copy's name starts with, how many
/// copies, and whether the statement is of a turned copy (k ≥ 1) rather than the representative.
#[derive(Clone)]
struct Ring {
    prefix: String,
    turned: bool,
}

/// A ring the walk expanded: what its copies' names start with, its centre (an absolute
/// reference once resolved), how many copies, and the block, for a message about it.
#[derive(Clone, Debug)]
pub struct RingInfo {
    pub prefix: String,
    pub about: Ref,
    pub n: u32,
    pub span: Span,
}

/// What the names in one statement are resolved against: the prefixes it is nested in, innermost
/// first.  An instance's entity arguments are not here: a formal is an *alias* under the
/// instance's own prefix (`bind`), found by `lookup` through the prefixes like any other name.
#[derive(Clone, Default)]
struct Scope {
    /// Lexical component, independent of temporary prefixes used for indexed lookup.
    owner: String,
    access: std::rc::Rc<BTreeMap<String, String>>,
    /// The groups an interface types (`Walk::typed`), by absolute key, to the interface's name:
    /// a field read past one names nothing it declares (`lookup`).  Given with `access`.
    typed: std::rc::Rc<BTreeMap<String, String>>,
    prefixes: Vec<String>,
    closed: bool,
    /// The names a closed component may not capture — see `Outside`.
    forbidden: Outside,
    /// The group and set names in force, shared as `vals` is.
    groups: std::rc::Rc<BTreeSet<String>>,
    /// The numbers in force where the statement was written — the enclosing counts, params and
    /// block binders.  An index (`p[i + 1]`) is an expression over exactly these, and references
    /// are resolved in a later pass where the walk's own environment is gone, so it travels here.
    /// Shared and not copied: every statement of a body carries the body's table, and the root's
    /// holds every instance's numbers (`P0.w`), so a copy per statement is quadratic (#403) —
    /// and a block's copy, a `when` or an application adds a frame over it, never a copy of it
    /// (#405, `Vals`).
    vals: std::rc::Rc<Vals>,
    /// The **sides** in force: a `Side` formal to the word it was given (`s` → `right`).  A side
    /// is a word and not a number (§9.2), so it travels in a table of its own rather than as a
    /// ±1 in `vals` — which is exactly the idiom issue #48's item 4 takes out of the helpers, and
    /// putting it back one level down would leave `s * 90deg` writable inside a body.
    sides: std::rc::Rc<BTreeMap<String, String>>,
    /// The file the body being walked was written in — the document (`None`) or a module — which
    /// is what a call's component name is resolved from (`Program::resolve_component`).
    module: Option<usize>,
    /// The blocks the statement stands in — see `InBlock`.
    block: InBlock,
    /// What the enclosing instances put on every declaration emitted under them — see `Stamps`.
    stamps: Stamps,
    /// How a predicate's body is being walked, where the statement is in one — see `PredWalk`.
    pred: PredWalk,
    /// Which reading at an instant the statement is one way of writing out (#391 S2), and which
    /// way: the index into `Walk::readings` and what its source must be for this to be the one
    /// kept (`carried`, `Walk::settle_readings`).
    reading: Option<(usize, carried::Read)>,
    /// **Where a definition's body stands** (#524): the `in`s around its instance's call and
    /// around every call enclosing it, outermost first — what `std.front` in the body names the
    /// datums of (`Lookup::datum`).  Empty at the root, the world.  `None` outside a definition's
    /// body: the document's own scope, a relation word's, a traced body's.
    stands: Option<std::rc::Rc<Vec<InPlane>>>,
}

/// The numbers in force outside a closed component's body (`Scope::forbidden`): the scope the
/// instance was written in and the document's, which a body that reads one by name has captured
/// rather than been handed (E103).  The tables themselves, shared, and asked at lookup: a set of
/// their names built per instance is a copy of the root's table per instance (#403).
#[derive(Clone, Default)]
struct Outside {
    around: std::rc::Rc<Vals>,
    file: std::rc::Rc<Vals>,
    /// The numbers of the file the body is written in that are or read an unknown, and so are
    /// no constants of it (#461 PC2): out of reach, said so (`Walk::file_numbers`).
    unknowns: std::rc::Rc<BTreeSet<String>>,
}

impl Outside {
    fn contains(&self, name: &str) -> bool {
        self.around.contains_key(name) || self.file.contains_key(name) || self.unknown(name)
    }

    /// Whether `name` is a number of the body's file, or of the document's, that is or reads an
    /// unknown.
    fn unknown(&self, name: &str) -> bool {
        self.unknowns.contains(name)
            || self.file.get(name).or_else(|| self.around.get(name)).is_some_and(|v| v.free.is_some())
    }
}

/// The blocks a statement stands in (`Scope::block`).
#[derive(Clone, Default)]
struct InBlock {
    cyc: Option<Cyc>,
    /// The `ring` the statement stands in, through however many instances and blocks.
    ring: Option<Ring>,
    /// Whether a `cycle` or a `repeat` stands anywhere above: the prefix in force then carries a
    /// block's id (`#3.0.`) rather than an instance's name, and a declaration under it is one
    /// *copy* — shown and selected by, never written into a statement.  This walk is the only
    /// place that is known (`syntax::Named`, issue #39).
    copies: bool,
}

/// What the enclosing instances put on every declaration emitted under them (`Scope::stamps`).
#[derive(Clone, Default)]
struct Stamps {
    roles: crate::semantics::GeometryRoles,
    /// The view an enclosing instance is drawn `in` (§6.7): every point-bearing declaration
    /// emitted under it joins the plane.  The ref as *written* at the instance, with the
    /// prefixes of the scope it was written in — `rewrite` resolves it there and not in the
    /// component's own chain, where a body declaration called `top` would take it (#45.4).
    plane: Option<InPlane>,
    /// The frames further out that an enclosing instance rides on (#383), outermost first, each
    /// as written where it was: what its statements ride on beyond `plane`.
    rides: Vec<InPlane>,
    /// The classes every enclosing instance was given (`t2 := Throw(…) class phantom`), which
    /// every declaration emitted under them carries beneath its own (§13.2).
    class: crate::style::Classes,
    /// Members of an unnamed call have internal keys but no path another statement can write.
    anonymous: bool,
}

/// How a predicate's body is being walked (§6.21), where a statement stands in one
/// (`Scope::pred`).
#[derive(Clone, Default)]
struct PredWalk {
    /// As itself; for what it makes and nothing it states (the geometry a tangency at a point
    /// reads its derivatives over); or a second time for a tangency, each relation emitted as
    /// its derivative and nothing made again.
    pass: apply::Pass,
    /// The statement a relation made by a predicate's use was written as in the body, kept when
    /// the application stamps the use's own over it (`apply`): what a set used there is applied
    /// under (`begin_nth`), so two uses in one body stand apart by what the source wrote.
    written: Option<crate::syntax::StmtId>,
    /// Under a set measured from (#183 F2): the point its body is applied to is a probe whose
    /// conditions are terms, so each equality a relation states is held as a bound at its number
    /// (`Written::held`, `apply`) — the one rule a region's term is read by (#192).  Carried into
    /// every use applied under it.
    held: bool,
}

impl Scope {
    /// The derivative a body walked for a tangency states its relations as (§6.21).
    fn twin(&self) -> Option<&crate::syntax::Along> {
        match &self.pred.pass {
            apply::Pass::Along(a) => Some(a),
            apply::Pass::Itself | apply::Pass::Made => None,
        }
    }

    /// A scope that reads absolute names, from the document's root, with `access`'s grants.
    fn absolute(access: std::rc::Rc<BTreeMap<String, String>>) -> Scope {
        Scope { prefixes: vec![String::new()], access, ..Scope::default() }
    }

    /// `inner` read where `ip` was written: its owner, prefixes, file and place (#45.4, #524).
    fn written_at(ip: &InPlane, inner: &Scope) -> Scope {
        Scope { owner: ip.owner.clone(), prefixes: ip.prefixes.clone(), closed: ip.closed,
                module: ip.module, stands: ip.stands.clone(), ..inner.clone() }
    }

    /// The innermost prefix — what a name declared here is put under.
    fn prefix(&self) -> &str {
        self.prefixes.first().map(String::as_str).unwrap_or("")
    }

    /// Whether this is the body of an instance declared in `ring` (#566), not the ring's own
    /// body or a ring inside a body.
    fn in_instance_of(&self, ring: &Ring) -> bool {
        self.closed && self.instance_prefix().starts_with(&ring.prefix)
    }

    /// The innermost *instance's* prefix — what a dotted name read inside a body is made
    /// absolute under (`t.w` in `u` is `u.t.w`).  A block copy's prefix (`#3.0.`) is skipped: a
    /// `cycle` is a repetition of statements written in the body around it.
    fn instance_prefix(&self) -> &str {
        self.prefixes.iter().map(String::as_str).find(|p| !is_copy_prefix(p)).unwrap_or("")
    }
}

/// Every number `{from}.…` in `vals`, copied into `sub` as `{to}.…`: what a group or a set
/// argument carries into the body it is handed to.
fn copy_members(vals: &Vals, from: &str, to: &str, sub: &mut BTreeMap<String, Aff>) {
    let start = format!("{from}.");
    for (k, v) in vals.under(&start) {
        sub.insert(format!("{to}.{}", &k[start.len()..]), v);
    }
}

/// Whether a prefix ends in a block copy's segments, `…#<statement>.<copy>.`.
fn is_copy_prefix(p: &str) -> bool {
    let mut segs = p.trim_end_matches('.').rsplit('.');
    let copy = segs.next().unwrap_or("");
    let block = segs.next().unwrap_or("");
    !copy.is_empty()
        && copy.bytes().all(|b| b.is_ascii_digit())
        && block.starts_with('#')
        && block[1..].bytes().all(|b| b.is_ascii_digit())
}

/// An instance's `in PLANE`, and where it was written — see `Stamps::plane`.
#[derive(Clone)]
struct InPlane {
    owner: String,
    plane: Ref,
    prefixes: Vec<String>,
    closed: bool,
    /// The file and the place it was written in (`Scope::{module, stands}`): what a standard
    /// datum it names is read by (#524).
    module: Option<usize>,
    stands: Option<std::rc::Rc<Vec<InPlane>>>,
}

impl InPlane {
    /// `plane`, written in `sc`.
    fn at(plane: &Ref, sc: &Scope) -> InPlane {
        InPlane { owner: sc.owner.clone(), plane: plane.clone(), prefixes: sc.prefixes.clone(),
                  closed: sc.closed, module: sc.module, stands: sc.stands.clone() }
    }
}

/// **The standard datums** (#524, #542), as a definition's body reads them: the files that say
/// `use std` — the document (`None`) or a module — empty where the drawing has none, and every
/// frame declared, by absolute name, whose own datums an instance standing in it reads.  `read`
/// is every frame a body has read a datum of that is made (`std.front`, not `std.x`, which is the
/// frame's `u`): its `FrameDatums`, whose declarations `own` are, walked once each after the
/// document (`Walk::finish`) under `keys::frame_datums`.
#[derive(Clone, Default)]
struct Datums {
    files: BTreeSet<Option<usize>>,
    frames: BTreeMap<String, Frame>,
    /// Shared by every clone (`drawn_incidences` resolves through one), so no read is lost.
    read: std::rc::Rc<std::cell::RefCell<BTreeSet<String>>>,
    own: BTreeMap<String, crate::model::EntKind>,
    /// **An instance read as a frame** (#548): every instance, by absolute name, whose `b.std.D`
    /// the scope declaring it or one around it writes (`framed::read_outside`) — each standing in
    /// a frame of its own, `keys::instance_frame` — and of those, the ones refused it because
    /// their body reads no datum, to the name of their definition.
    outside: BTreeSet<String>,
    refused: BTreeMap<String, String>,
    /// **The mirrored instances** (#578), by prefix (`left.`): what a reference from outside
    /// reads under one is its image (`keys::image`), and its `b.std.D` its image's datums
    /// (`keys::image_datum`).
    mirrored: BTreeSet<String>,
}

/// A frame declared (#524, #542): a reading at an instant (`Walk::walk_reading_at`) is one where
/// its source, read where it was written, is — `None` for one declared bare — and its datums are
/// walked where it was, so what a part declares it in makes them.
#[derive(Clone)]
struct Frame {
    reads: Option<(Ref, Scope)>,
    path: Vec<PathStep>,
}

pub struct Expansion {
    pub private_names: BTreeMap<String, String>,
    pub flat: Vec<crate::ir::Statement>,
    pub diagnostics: Vec<Diag>,
    /// Every instance the walk bound, drawn or not — what a curve is a curve *of* (§6.5): the
    /// elaborator finds the instance a curve's point belongs to here, with the entities it was
    /// given resolved to absolute names and the numbers it was given worked out.
    pub instances: Vec<InstanceInfo>,
    /// Every formal bound to an actual, resolved: absolute name to absolute name.  What a
    /// curve's point is when the name it was asked by is an alias.
    pub aliases: BTreeMap<String, String>,
    /// Every unknown the document declared, by absolute name: an input nothing binds (`param
    /// beta: Angle`) and a formal a call left unbound with a seed (`Wing(f, beta: hint(15deg))`)
    /// — what each is and where its solve begins (`model::Declared`).
    pub unknowns: BTreeMap<String, crate::model::Declared>,
    /// Every parameter axis the document declared (`param t: Angle over (a, b)`, #391 S1), by
    /// name: a number that runs over its interval, read by nothing yet.
    pub parameters: BTreeMap<String, crate::model::Running>,
    /// Every material the document states (`steel := material(E: 200GPa, …)`, #510 P3), by
    /// absolute name.
    pub materials: BTreeMap<String, crate::model::Material>,
    /// Every number the document names and the walk worked out (#596), by absolute name — see
    /// `Valued`.
    pub values: BTreeMap<String, Valued>,
    /// Every claim over values, judged, in the order walked (#596) — see `ValueClaimed`.
    pub value_claims: Vec<ValueClaimed>,
    /// Every `ring` expanded, its centre resolved — what the elaborator turns copies about.
    pub rings: Vec<RingInfo>,
    /// Every set drawn as an element, for the elaborator to judge (`lowering`, #105).
    pub lowered: Vec<Lowered>,
    /// Every point drawn in further planes and each of its twins, by absolute key (§6.7).
    pub twins: Vec<(String, String)>,
    /// What every reference reading a member `normal` reads it of, absolute (`P.normal` gives
    /// `P`): the planes among them mint their normal as they are built
    /// (`Sketch::plane_normal`), and the rest are said to have none where they are read.
    pub normals: BTreeSet<String>,
    /// What every reference reading a member `u`, `v`, `origin` or `normal` reads it of,
    /// absolute (#383): a frame declared bare among them mints its axes as it is built, and
    /// one not among them owns nothing.
    pub charts: BTreeSet<String>,
    /// What every reference reading a joint's member `axis`, `ratio` or `advance` reads it of, and
    /// every frame a `fix` holds a ratio or an advance of, absolute (#383): a frame among them
    /// mints its joint as it is built.
    pub joints: BTreeSet<String>,
    /// Where a declaration reads through a name already said to name nothing — a formal its
    /// call gave nothing that is (#253): the elaborator, finding nothing there, says no more.
    pub quiet: BTreeSet<Span>,
    /// Every region solid's probe, by absolute key: the point its set is applied to (§6.21).
    pub probes: Vec<String>,
    /// The solids a block runs over the faces of, whose faces were not given (#183 F4): the
    /// elaborator names them once it has built the solids, and expands again.
    pub face_repeats: BTreeSet<String>,
    /// The plane each face of a solid used as a set is drawn in, under the hidden name its
    /// carrier reads it by (`faces`), for the elaborator to bind once memberships are in.
    pub face_planes: Vec<FacePlane>,
    /// The hidden axis each side used as a set is level along, for the elaborator to seed.
    pub face_axes: Vec<FaceAxis>,
    /// Every `path` formal bound (#251) and every member an interface asks to be an entity of a
    /// kind (§8.1 [0.84]), as given: for the elaborator to judge once it knows what each name is
    /// (E103).
    pub(crate) kinds: Vec<KindAsked>,
    /// Every face family's running number (#243 L1), by absolute name (`#f12.s`): its dimension
    /// and the number the drawn station stands at, the middle of its interval — see `Held`.
    pub held: BTreeMap<String, (crate::units::Dim, f64)>,
    /// Every number of the geometry a face family's station reads in a dimension (#243 L1), as
    /// its traced body would: the name the text reads it by (`#f12.g.p2.y`), the declaration it
    /// resolved to and the fields past it — for the elaborator to hold as the geometry stands.
    pub reads: Vec<(String, String, Vec<String>)>,
    /// Every pair of faces a `when` left apart (#625): faces it read as meeting nowhere where the
    /// drawing is seeded, which made the copy nothing — for the elaborator to read again where
    /// it is solved (`program::meets`).
    pub apart: Vec<Apart>,
    /// Whether the walk stopped at `MAX_FLAT`, its statements cut short: no drawing.
    pub overflowed: bool,
}

/// **Two faces of a solid a `when … meets …` left apart** (#625): the solid by its absolute
/// name, the faces by theirs from it, and the document's statement that made the block.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Apart {
    pub solid: String,
    pub faces: [String; 2],
    pub at: Span,
}

/// **A number the document names** (#596): `ratio := teeth / 12 * 4`, a `param`, a group's
/// member, as the walk worked it out — of its dimension, in the document's own units (a length in
/// its unit, the bases beyond length and angle in SI's).  Only a number: one affine in an unknown
/// of the drawing is the solve's to give, and is not here.
#[derive(Clone, Debug)]
pub struct Valued {
    pub value: f64,
    pub dim: crate::units::Dim,
}

/// **A claim over values, judged** (#596): `claim torque >= 2Nm` where it stands, its text as
/// written, whether it holds, and each value it reads by absolute name, as it was worked out.  A
/// value is worked out before the solve, so the claim is a theorem or refuted, never consuming;
/// one that cannot be decided is a diagnostic, and is not here.
#[derive(Clone, Debug)]
pub struct ValueClaimed {
    pub text: String,
    pub span: Span,
    pub holds: bool,
    pub reads: Vec<(String, Valued)>,
}

/// **A face family's running number** (#243 L1), as the walk carries it: a free value named
/// after the formal (`#f12.s`, the unbound formal `bind` makes), which a text affine in it
/// carries as any unknown's, and a text that is not — `twist * (3 * s^2 - 2 * s^3)` — is
/// **kept** and written in where it is read, as a trace keeps one (`Sym`).  It is no unknown:
/// the sketch holds it at a number (`Sketch::held`), the station the family solves at.
#[derive(Default)]
struct Held {
    /// Each running number, its dimension and where the drawn station stands.
    names: BTreeMap<String, (crate::units::Dim, f64)>,
    /// Params and arguments that came to text over one, by absolute name.
    texts: BTreeMap<String, String>,
    /// The numbers of the geometry the stations read — see `Expansion::reads`.
    reads: Vec<(String, String, Vec<String>)>,
    /// Each face family run over a named chain (#243 L4), by its instance's absolute key: one
    /// leg per link its interval reaches, each the leg's running number and the link's index.
    chained: BTreeMap<String, Vec<(String, f64)>>,
    /// Each leg's running number: the link's index — where the leg's `s` starts, so the
    /// family's formal reads `k + s` there — and the absolute names of its links, one per chain
    /// run in step, on which a contact pinned to `s` reads the leg's own number.
    legs: BTreeMap<String, (f64, Vec<String>)>,
    /// Each face family's running formal, by its instance's absolute key (#398 B): the one the
    /// call gives the parameter it runs over, or the one named as it.
    formals: BTreeMap<String, String>,
    /// The leg whose link the relation being settled names, where it names it: a pin to `s`
    /// there is the leg's running number (`settle_relation`).
    on_leg: Option<(String, f64)>,
}

/// An entity a call gave where a kind (or any `path`) is asked for: the absolute name of what it
/// is, where the argument is written, what is asked, and who asks — a `path` formal by its name
/// (`interface` `None`), or an interface's member as the call reaches it (`gear_cone.generator`).
#[derive(Clone, Debug)]
pub(crate) struct KindAsked {
    pub target: String,
    pub span: Span,
    pub ty: crate::syntax::Ty,
    pub what: String,
    pub interface: Option<String>,
}

/// One instance, as bound: which component, under what prefix, given what.
#[derive(Clone, Debug)]
pub struct InstanceInfo {
    /// The absolute prefix every name of its expansion starts with — `leg.`, `g.t.r.`, or
    /// `#c12.` for an instance written in place inside a curve.
    pub prefix: String,
    /// The component as the call wrote it (`Tooth`, `engine.parts.Crank`).
    pub component: String,
    /// Which of the program's components it is.
    pub comp: usize,
    /// The entity formals in order, each with the absolute name of the actual it aliases —
    /// `None` where the instance gave none, or gave one that resolved to nothing.
    pub ents: Vec<(String, Option<String>)>,
    /// The numeric formals, by name: a number, or the drawing's unknown a formal left unbound
    /// became (a free `Aff`, `bind`).
    pub values: BTreeMap<String, Aff>,
    /// Whether its body was expanded onto the sheet.  An instance written in place inside a
    /// curve is bound and never drawn: the curve is the only thing made of it.
    pub drawn: bool,
    /// The instances and blocks enclosing its body's statements, this one last — the path every
    /// site of what it made starts with (`program::InstPath`).  Empty for one never expanded.
    pub path: Vec<PathStep>,
    /// What it is a function of, where it is closed (#566): its binding's key and the frame it
    /// stands in.  `None` for any other instance.
    pub binding: Option<Bound>,
    /// Where the call is mirrored, `mirrored(front)` (#578).  `None` for any other.
    pub mirror: Option<Mirroring>,
}

/// **A mirrored call** (#578): the axis of the frame it stands in its image reverses
/// (`syntax::Mirrored::axis`), where the trailer is written, and where the call gives what a
/// closed instance is not given — an entity, a group or an unknown — which a refusal points at.
#[derive(Clone, Debug)]
pub struct Mirroring {
    pub axis: u8,
    pub span: Span,
    pub given: Option<Span>,
}

/// A block over a chain's edges, set aside by the walk until the chain it names can be found:
/// its count is the chain's length, and the chain may stand further down or inside an instance
/// the walk has not reached (P2).  The block statement itself stands in the walk's output where
/// it was met, at its own path, and its copies replace it there.
struct Pending {
    st: Stmt,
    scope: Scope,
    path: Vec<PathStep>,
    depth: usize,
    /// The document's statement it was met under (`FaceState::at`).
    at: Span,
}

/// One link of a walk a block runs over (#243 L8).
#[derive(Clone)]
enum Link {
    /// An edge written — a chain's link, or an edge a face names — with the scope it was
    /// written in.
    Edge(Ref, Scope),
    /// The straight run a face of corners makes from one corner to the next, the corners read
    /// in the face's scope: a line drawn once under the face's key for it (`keys::run`), which
    /// the source cannot name.
    Run { from: Ref, to: Ref, scope: Scope, key: String },
}

/// The walk's *symbolic* mode: a component expanded over its formals as **variables**, which is
/// what compiling a curve over it needs (§6.5).  The numeric formals are bound as free values
/// named after themselves, so the ordinary machinery carries them — `substitute` writes a free
/// value back out by name, `settle` keeps a dimension that comes to no number — and the mode
/// adds one policy: a text that cannot be worked out at all (a `param` or a seed over `sin(u)`,
/// say) is **kept** as text and written in where it is read, where the sheet would report it.
#[derive(Default)]
struct Sym {
    /// Params and arguments that came to text rather than a value, by absolute name: `(sin(u))`.
    texts: BTreeMap<String, String>,
}

struct Walk<'a> {
    private_names: BTreeMap<String, String>,
    prog: &'a Program,
    /// What the document's numbers are in — carried through the walk because every expression
    /// worked out here is worked out in them.
    units: Units,
    out: Vec<(Stmt, Vec<PathStep>, Scope)>,
    /// Every absolute name a declaration will make.  Collected as the walk goes and used to
    /// resolve references afterwards, so forward reference works — which spec P2 requires, since
    /// a body is a set and a set has no "before".
    names: BTreeSet<String>,
    /// Those of them a solid's declaration makes: a reference past one may name one copy's faces
    /// (`flange.drill[1]`, `Lookup::face_of_copy`).
    solid_names: BTreeSet<String>,
    /// A formal bound to an actual: one name for what another names.  Resolved transitively,
    /// after the walk.
    aliases: Vec<(String, Ref, Scope)>,
    /// Each key of `aliases` with its entries' indices in order, kept up to as many of them as
    /// `resolve::Lazy` last read (only ever added to, so brought up to date from there).
    alias_keys: std::cell::RefCell<(usize, BTreeMap<String, Vec<usize>>)>,
    instances: Vec<InstanceInfo>,
    /// `Some` while a component is expanded over its formals as variables — see `Sym`.
    sym: Option<Sym>,
    /// Root values, used to diagnose accidental capture by a closed component.
    file_vals: std::rc::Rc<Vals>,
    /// Module values exported to importing root bodies; component bodies receive arguments.
    module_vals: Vec<Option<BTreeMap<String, Aff>>>,
    /// The constants each file's definitions read (#461 PC1), by the file (`None` the
    /// document's), worked out once each, with the names of its numbers that are none: what
    /// `file_numbers` gives.
    constants: BTreeMap<Option<usize>, (std::rc::Rc<Vals>, std::rc::Rc<BTreeSet<String>>)>,
    diagnostics: Vec<Diag>,
    /// Whether the walk has made `MAX_FLAT` statements, said once (`Expansion::overflowed`).
    overflowed: bool,
    /// Every call already read for how its arguments are written (`check_call`), by where it is
    /// written.  The question is about the *text* — which formal a written argument lands on — so
    /// it is asked once per call however many times the walk binds it: thirty copies of a `cycle`
    /// are one mistake.  A call the core wrote itself (`edit::add_rectangle`'s instance, a curve's
    /// arguments) carries no span and so is read at most once, which costs nothing: it labels
    /// every number by construction.
    called: BTreeSet<Span>,
    /// The standard datums (`StandardDatums`): who reads them, where a file read says `use std`,
    /// and the frames that refuse them — see `Datums`.
    standard: Datums,
    group_names: BTreeSet<String>,
    /// The names a definition gives a group by naming it (`small := sizes`, or a member
    /// `{bar: bar}`, #594), by absolute key, to the group's: an alias of that group, never a
    /// group of its own.
    group_aliases: BTreeMap<String, String>,
    /// The groups the source defines in braces (`g := {…}`, and one in place in it), by absolute
    /// key: of `group_names`, the ones that are no instance, no typed formal and no argument.
    defined_groups: BTreeSet<String>,
    /// Every interface of the program, resolved once (`interfaces::shapes`).
    shapes: Vec<std::rc::Rc<interfaces::Shape>>,
    /// The groups interfaces type (`Walk::typed`), by absolute key, to the interface's name.
    typed: BTreeMap<String, String>,
    /// What a binding or a read asks that only every name known can answer — see `Checks`.
    checks: Checks,
    /// Blocks set aside until their chain is found — see `Aside`.
    aside: Aside,
    /// The unknowns declared so far — see `Expansion::unknowns`.
    unknowns: BTreeMap<String, crate::model::Declared>,
    /// The parameter axes declared so far — see `Expansion::parameters`.
    parameters: BTreeMap<String, crate::model::Running>,
    /// The materials declared so far — see `Expansion::materials` — or `None` for one whose
    /// properties were refused: still a name a solid's `of` may give, already said to be wrong.
    materials: BTreeMap<String, Option<crate::model::Material>>,
    /// The values worked out and the claims over them judged so far — see `Expansion::values`
    /// and `Expansion::value_claims` — the claims by where each stands and the prefix it is
    /// walked under, so a body walked again (§6.21) judges nothing twice.
    values: BTreeMap<String, Valued>,
    value_claims: Vec<ValueClaimed>,
    claims_judged: BTreeSet<(Span, String)>,
    /// Every reading at an instant walked and not yet settled (#391 S2): what it reads, and the
    /// scope that names it in — see `carried`.
    readings: Vec<(Ref, Scope)>,
    /// The components being expanded, outermost first, each with the numbers it was given: a
    /// call to one already here is a component instantiating itself, walked like any other call
    /// (a `when` ends it, §12.9) unless it is given the numbers it was given there, which make
    /// the same choices again and so never end (E003).
    instantiating: Vec<(&'a Component, std::rc::Rc<Vals>)>,
    /// The rings expanded — see `Rings`.
    rings: Rings,
    /// The relation words applied and what applications bound — see `WordState`.
    words: WordState,
    /// The sets made and how they are drawn — see `SetState`.
    sets: SetState<'a>,
    /// Every point drawn in further planes and each of its twins — see `Expansion::twins`.
    twins: Vec<(String, String)>,
    /// See `Expansion::normals`.
    normals: BTreeSet<String>,
    /// See `Expansion::charts`.
    charts: BTreeSet<String>,
    /// See `Expansion::joints`.
    joints: BTreeSet<String>,
    /// See `Expansion::quiet`.
    quiet: BTreeSet<Span>,
    /// The solids' faces read as sets and blocks over them — see `FaceState`.
    faces: FaceState,
    /// The face families' running numbers — see `Held`.
    held: Held,
    /// What the walk found to be one thing stated twice (#415) — see `same`.
    same: same::Same,
}

/// What a binding or a read asks that only every name known can answer (`resolve`), and the
/// kinds it asked for (`Walk::checks`).
#[derive(Default)]
struct Checks {
    /// A group or set formal's alias key, where it was given, and which of the two it wants:
    /// checked against what the alias reaches once every name is known (`resolve`).
    groups: Vec<(String, Span, crate::syntax::Ty)>,
    /// A `path` formal's alias key, where it was given and the formal's name: resolved with the
    /// rest (`resolve`) into `Expansion::kinds`.
    paths: Vec<(String, Span, String)>,
    /// A formal an interface types: the key of the whole argument's alias, where it was given,
    /// the shape, and the argument as written — checked against what the alias reaches
    /// (`resolve`, `conform`).
    interfaces: Vec<(String, Span, std::rc::Rc<interfaces::Shape>, String)>,
    /// What path formals and interfaces asked to be an entity of a kind, for `Expansion::kinds`.
    kinds: Vec<KindAsked>,
    group_fields: Vec<(String, Span)>,
    /// The dotted names a dimension's text reads that nothing numeric in scope answers to, with
    /// where each was written and the scope to resolve it in: judged once every name is known
    /// (`resolve`), since only then can `k.r` (a circle's radius) be told from `t.w` (a named
    /// dimension of the instance `t`).
    /// A station's text kept whole (`Walk::station_text`) carries the key each read is written
    /// in as, its copy's own (`#3.0.e.p1.x`), so the corners of a block's copies stay apart.
    dim_reads: Vec<(String, Span, Scope, Option<String>)>,
    /// The absolute names a number given in a face family's station already reads (#243 L8,
    /// `Walk::station_text`): its geometry's, set aside where the number is written, and the
    /// unknowns it is affine in.  A text they are written into is settled with them as they are.
    settled: BTreeSet<String>,
    /// Where a statement read a name nothing declares (E101): the statement is not emitted, so
    /// the name never reaches the expression graph to become an unknown of its own making.
    refused: Vec<Span>,
}

/// Blocks over a chain's edges, set aside until their chain is found (`Walk::aside`).
#[derive(Default)]
struct Aside {
    /// The blocks waiting for their chain — see `Pending`.
    blocks: Vec<Pending>,
    /// The splines through a chain's corners, waiting for every chain — see `through::Fit`.
    fits: Vec<through::Fit>,
    /// Statements held aside in the walk's own output while a deferred block is expanded into a
    /// vector of its own, so the statement cap counts them.
    held: usize,
}

/// The rings the walk expanded (`Walk::rings`).
#[derive(Default)]
struct Rings {
    /// Waiting for their centres to be resolved with everything else: the copies' prefix, the
    /// centre as written and the scope it was written in, the count, the block.
    written: Vec<(String, Ref, Scope, usize, Span)>,
    /// Those resolved — see `Expansion::rings`.
    infos: Vec<RingInfo>,
}

/// The relation words applied, and what a call's or a use's application bound
/// (`Walk::words`).
#[derive(Default)]
struct WordState {
    /// Each relation word reached, read once — see `expand_word`.
    kinds: BTreeMap<usize, Option<std::rc::Rc<BTreeMap<String, words::Param>>>>,
    /// The relation words being applied, outermost first, each with where its outermost use is
    /// written: one reached again inside its own application is defined in terms of itself.
    applying: Vec<(usize, Span)>,
    /// The names an application bound to what its use wrote (`apply::bind_to_use`): one that
    /// names nothing is said once, at the operand, and not again inside the body.
    use_aliases: BTreeSet<String>,
    /// The names bound to what a call or a use wrote: an instance's entity formals and an
    /// application's bound variables.  A ring's representative is judged by the argument as
    /// written (E021), and not again where a body reads the name (#226).
    arguments: BTreeSet<String>,
    /// The applications whose body declares something, by prefix: an operand read there may be
    /// built on rather than related, which a ring's instance may not do with its frame's datums
    /// (#566).
    declaring: BTreeSet<String>,
}

/// The sets the walk made (§6.21) and how each is drawn (`Walk::sets`).
struct SetState<'a> {
    /// Every set the walk made, by absolute name — see `sets::Site`.
    made: BTreeMap<String, sets::Site>,
    /// Those drawn as elements (`lowering`), by the same name, with what each is judged by.
    lowered: BTreeMap<String, crate::lowering::Form>,
    /// The sets the elaborator refused as elements, walked as sets (`expand_with`).
    unlowered: &'a BTreeSet<String>,
    /// Whether the uses of sets are being applied, so a set made now is made inside one.
    applying: bool,
    /// Every region solid's probe — see `Expansion::probes`.
    probes: Vec<String>,
}

/// **A solid's faces as the elaborator names them** (#183 F4): each by the path it is named by
/// from the solid, and which of them meet (#625, `Meets`), where its boundary says.
#[derive(Clone, Debug)]
pub struct Listed {
    pub paths: Vec<crate::solid::faces::FacePath>,
    pub meets: Option<crate::solid::faces::Meets>,
}

/// The solids' faces read as sets, and the blocks over them (`Walk::faces`).
#[derive(Default)]
struct FaceState {
    /// Each solid's faces, where the elaborator has said (#183 F4) — see
    /// `Expansion::face_repeats`.
    given: BTreeMap<String, Listed>,
    repeats: BTreeSet<String>,
    /// See `Expansion::apart`.
    apart: BTreeSet<Apart>,
    /// The document's own statement being walked — its root body's — where a block set aside
    /// was met: what a fault deep in what it expands to is said at (#625).
    at: Span,
    /// See `Expansion::face_planes` and `face_axes`.
    planes: Vec<FacePlane>,
    axes: Vec<FaceAxis>,
}

/// Expand a program's root component into a flat list of declarations, constraints, gauges and
/// orientations, with every name made absolute.
pub fn expand(prog: &Program, units: Units) -> Expansion {
    expand_with(prog, units, &BTreeSet::new())
}

/// `expand`, with the sets named in `unlowered` walked as sets though their bodies are elements'
/// — those the elaborator refused (`lowering::refused`).
pub fn expand_with(prog: &Program, units: Units, unlowered: &BTreeSet<String>) -> Expansion {
    expand_faces(prog, units, unlowered, &BTreeMap::new())
}

/// `expand_with`, each solid a block runs over the faces of given its faces (#183 F4).
pub fn expand_faces(
    prog: &Program,
    units: Units,
    unlowered: &BTreeSet<String>,
    faces: &BTreeMap<String, Listed>,
) -> Expansion {
    let mut w = Walk::new(prog, units, None);
    w.sets.unlowered = unlowered;
    w.faces.given = faces.clone();
    let root = prog.root();
    // A definitions-only file has nothing to expand. In particular, imported parameters must
    // not be evaluated in a unit system that only an eventual instantiating model will supply.
    if root.body.is_empty() {
        return w.finish();
    }
    let scope = Scope { prefixes: vec![String::new()], ..Scope::default() };
    w.body(&root.body, &scope, &mut Vals::default(), &[], 0);
    w.finish()
}

/// Expand one component **over its formals** — the form a curve is compiled from (§6.5).
///
/// The entity formals are names the body may reach (`c`, `c.center`), bound to nothing: what
/// they denote is the curve's own business, a column of its variable table per coordinate.  The
/// numeric formals are variables too, so nothing that reads one is worked out — see `Sym`.  What
/// comes out is the body as a flat list of statements under an empty prefix, exactly as a drawn
/// instance's would come out under its own, with `param`s, nested instances and repetition all
/// done here and not again by the compile.
pub fn expand_component(prog: &Program, comp: &Component, units: Units) -> Expansion {
    let mut w = Walk::new(prog, units, Some(Sym::default()));
    let scope = Scope {
        prefixes: vec![String::new()],
        closed: true,
        module: comp.module,
        ..Scope::default()
    };
    let mut vals = Vals::default();
    for f in &comp.formals {
        match f.ty {
            Ty::Ent(_) | Ty::Path | Ty::Group | Ty::Set => {
                w.names.insert(f.name.text.clone());
            }
            // a variable of the curve: a free value named after itself, which the ordinary
            // walk carries into every text that reads it
            ty => {
                vals.insert(f.name.text.clone(), free(f.name.text.clone(), ty));
            }
        }
    }
    w.body(&comp.body, &scope, &mut vals, &[], 1);
    w.finish()
}

impl<'a> Walk<'a> {
    fn new(prog: &'a Program, units: Units, sym: Option<Sym>) -> Walk<'a> {
        let std_module = prog.modules.iter().position(|m| m.name == "std");
        let standard = std_module.and_then(|k| prog.component_in(Some(k), "StandardDatums"));
        let declared = |c: &'a Component| c.body.iter().filter_map(|st| match &st.kind {
            StmtKind::Decl(d) => Some((d.name.key().text.clone(), d.kind)),
            _ => None,
        });
        let shadowed = prog.root().body.iter().any(|st| match &st.kind {
            StmtKind::Instance(i) => i.name.text == "std",
            StmtKind::Decl(d) => d.name.key().text == "std",
            StmtKind::Group(g) => g.name.text == "std",
            _ => false,
        });
        // the datums are the standard library's names like any other (§14.4): a file reaches
        // `std.front` through a `use std` of its own — the document at its root and in its
        // definitions, a module in its definitions (#524) — and they are made where any does
        let made = sym.is_none() && standard.is_some() && !shadowed;
        let files: BTreeSet<Option<usize>> = std::iter::once(None)
            .chain((0..prog.modules.len()).map(Some))
            .filter(|&f| made && prog.uses_of(f).contains(&"std"))
            .collect();
        Walk {
            private_names: BTreeMap::new(),
            prog,
            units,
            out: Vec::new(),
            names: if !files.is_empty() {
                declared(standard.unwrap()).map(|(n, _)| format!("std.{n}")).collect()
            } else { BTreeSet::new() },
            solid_names: BTreeSet::new(),
            aliases: Vec::new(),
            alias_keys: Default::default(),
            instances: Vec::new(),
            sym,
            file_vals: Default::default(),
            module_vals: vec![None; prog.modules.len()],
            constants: BTreeMap::new(),
            diagnostics: Vec::new(),
            overflowed: false,
            called: BTreeSet::new(),
            standard: Datums {
                files,
                own: std_module.and_then(|k| prog.component_in(Some(k), "FrameDatums"))
                    .into_iter().flat_map(declared).collect(),
                ..Datums::default()
            },
            group_names: BTreeSet::new(),
            group_aliases: BTreeMap::new(),
            defined_groups: BTreeSet::new(),
            shapes: interfaces::shapes(prog),
            typed: BTreeMap::new(),
            checks: Checks::default(),
            aside: Aside::default(),
            unknowns: BTreeMap::new(),
            parameters: BTreeMap::new(),
            materials: BTreeMap::new(),
            values: BTreeMap::new(),
            value_claims: Vec::new(),
            claims_judged: BTreeSet::new(),
            readings: Vec::new(),
            instantiating: Vec::new(),
            rings: Rings::default(),
            words: WordState::default(),
            sets: SetState {
                made: BTreeMap::new(),
                lowered: BTreeMap::new(),
                unlowered: &NONE,
                applying: false,
                probes: Vec::new(),
            },
            twins: Vec::new(),
            normals: BTreeSet::new(),
            charts: BTreeSet::new(),
            joints: BTreeSet::new(),
            quiet: BTreeSet::new(),
            faces: FaceState::default(),
            held: Held::default(),
            same: same::Same::default(),
        }
    }

    fn err(&mut self, code: Code, span: Span, message: impl Into<String>) {
        self.diagnostic(Diag::new(code, span, None, message.into()));
    }

    /// An error said once at its place, however many times the walk passes it — a block in a
    /// component instantiated thirty times is one mistake.
    fn once(&mut self, code: Code, span: Span, message: impl Into<String>) {
        if !self.diagnostics.iter().any(|d| d.code == code && d.span == span) {
            self.err(code, span, message);
        }
    }

    fn diagnostic(&mut self, diag: Diag) {
        if self.diagnostics.len() < 200 {
            self.diagnostics.push(diag);
        }
    }

    /// The standard datums' component and the scope its statements are walked in, where the
    /// drawing has them (a `use std` in any file read).
    fn datums(&self) -> Option<(&'a Component, Scope)> {
        if self.standard.files.is_empty() {
            return None;
        }
        let comp = self.std_component("StandardDatums")?;
        Some((comp, Scope { prefixes: vec!["std.".into()], module: comp.module, ..Scope::default() }))
    }

    /// `std`'s component `name`.
    fn std_component(&self, name: &str) -> Option<&'a Component> {
        self.prog.component_in(self.prog.modules.iter().position(|m| m.name == "std"), name)
    }

    /// Frame `abs`, declared at `path` (`Datums::frames`): its own datums' names made with it, a
    /// key no source can spell, so a name only to a body reading them as `std` (#542).
    fn declare_frame(&mut self, abs: String, reads: Option<(Ref, Scope)>, path: &[PathStep]) {
        let made = keys::frame_datums(&abs);
        self.names.extend(self.standard.own.keys().map(|d| format!("{made}{d}")));
        self.standard.frames.insert(abs, Frame { reads, path: path.to_vec() });
    }

    /// **A frame's own standard datums** (#542), for every frame a body standing in it read one
    /// of (`Datums::read`): `std`'s `FrameDatums` walked under `keys::frame_datums`, its formal
    /// the frame, once a frame and where the frame is declared — after the document's own and the
    /// world's, which resolution by name lets every body read whatever order they are made in.
    fn frame_datums(&mut self) {
        let Some(comp) = self.std_component("FrameDatums") else { return };
        let [formal] = &comp.formals[..] else { return };
        let frames = self.standard.read.borrow().clone();
        for frame in frames {
            let prefix = keys::frame_datums(&frame);
            let root = Scope::absolute(Default::default());
            let given = Ref::dotted(&frame, formal.name.span);
            self.aliases.push((format!("{prefix}{}", formal.name.text), given, root));
            // named by no name a reader sees: they are read only as a body's `std`
            let mut scope = Scope { prefixes: vec![prefix], module: comp.module, ..Scope::default() };
            scope.stamps.anonymous = true;
            let mut vals = Vals::of(self.module_params(comp.module.unwrap()));
            let path = self.standard.frames.get(&frame).map(|f| f.path.clone()).unwrap_or_default();
            self.body(&comp.body, &scope, &mut vals, &path, 1);
        }
    }

    fn finish(mut self) -> Expansion {
        self.expand_pending();
        self.expand_fits();
        // what each reading at an instant reads, once every name is known (#391 S2): a solid's
        // faces are named of what it is
        self.settle_readings();
        // a set's uses, once every set is known (§6.21) — a solid's faces among them — and a use
        // may hold a block over a chain
        self.face_sets();
        self.expand_sets();
        self.expand_pending();
        self.expand_fits();
        let (mut flat, mut aliases) = self.resolve();
        let lowered = self.to_judge(&aliases);
        if let Some((comp, scope)) = self.datums() {
            // A document that says `use std` has the standard datums, as a CAD part has its
            // origin planes, whether or not anything refers to them yet: the workspace offers
            // them as places to draw.  They are ordinary library statements with their own spans.
            // They follow the document's own, so its points keep the indices they would have
            // without them; resolution is by name, so nothing reads them the worse for it.
            let mut vals = Vals::of(self.module_params(comp.module.unwrap()));
            self.body(&comp.body, &scope, &mut vals, &[], 1);
            self.frame_datums();
            self.expand_pending();
            let (mut datums, resolved) = self.resolve();
            flat.append(&mut datums);
            aliases = resolved;
        }
        Expansion {
            private_names: self.private_names,
            flat,
            diagnostics: self.diagnostics,
            instances: self.instances,
            aliases,
            unknowns: self.unknowns,
            parameters: self.parameters,
            materials: self.materials.into_iter().filter_map(|(k, m)| Some((k, m?))).collect(),
            values: self.values,
            value_claims: self.value_claims,
            rings: self.rings.infos,
            lowered,
            twins: self.twins,
            normals: self.normals,
            charts: self.charts,
            joints: self.joints,
            quiet: self.quiet,
            probes: self.sets.probes,
            face_repeats: self.faces.repeats,
            face_planes: self.faces.planes,
            face_axes: self.faces.axes,
            kinds: self.checks.kinds,
            held: self.held.names,
            reads: self.held.reads,
            apart: self.faces.apart.into_iter().collect(),
            overflowed: self.overflowed,
        }
    }
}
