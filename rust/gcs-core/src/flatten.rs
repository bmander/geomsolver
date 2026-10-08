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

pub(crate) use words::faults as word_faults;
pub use crate::lowering::{Form, Lowered};

pub(crate) use values::{substitute_with, value_aff};
use values::{free, typed, value_of, substitute, reads_geometry};
use resolve::{lookup, lookup_raw, written};

/// No set walked as a set for want of an element: `Walk::unlowered` where nothing was.
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

/// A reference spelled back as one name, as the source wrote it — for a message about it.
pub fn written_name(r: &Ref) -> String {
    written(r)
}

/// What the names in one statement are resolved against: the prefixes it is nested in, innermost
/// first.  An instance's entity arguments are not here: a formal is an *alias* under the
/// instance's own prefix (`bind`), found by `lookup` through the prefixes like any other name.
#[derive(Clone, Default)]
struct Scope {
    /// Lexical component, independent of temporary prefixes used for indexed lookup.
    owner: String,
    access: std::rc::Rc<BTreeMap<String, String>>,
    in_roles: crate::semantics::GeometryRoles,
    prefixes: Vec<String>,
    closed: bool,
    forbidden: BTreeSet<String>,
    groups: BTreeSet<String>,
    cyc: Option<Cyc>,
    /// The `ring` the statement stands in, through however many instances and blocks.
    ring: Option<Ring>,
    /// Whether a `cycle` or a `repeat` stands anywhere above: the prefix in force then carries a
    /// block's id (`#3.0.`) rather than an instance's name, and a declaration under it is one
    /// *copy* — shown and selected by, never written into a statement.  This walk is the only
    /// place that is known (`syntax::Named`, issue #39).
    copies: bool,
    /// Members of an unnamed call have internal keys but no path another statement can write.
    anonymous: bool,
    /// The numbers in force where the statement was written — the enclosing counts, params and
    /// block binders.  An index (`p[i + 1]`) is an expression over exactly these, and references
    /// are resolved in a later pass where the walk's own environment is gone, so it travels here.
    vals: BTreeMap<String, Aff>,
    /// The view an enclosing instance is drawn `in` (§6.7): every point-bearing declaration
    /// emitted under it joins the plane.  The ref as *written* at the instance, with the
    /// prefixes of the scope it was written in — `rewrite` resolves it there and not in the
    /// component's own chain, where a body declaration called `top` would take it (#45.4).
    in_plane: Option<InPlane>,
    /// The classes every enclosing instance was given (`t2 := Throw(…) class phantom`), which
    /// every declaration emitted under them carries beneath its own (§13.2).
    in_class: crate::style::Classes,
    /// The **sides** in force: a `Side` formal to the word it was given (`s` → `right`).  A side
    /// is a word and not a number (§9.2), so it travels in a table of its own rather than as a
    /// ±1 in `vals` — which is exactly the idiom issue #48's item 4 takes out of the helpers, and
    /// putting it back one level down would leave `s * 90deg` writable inside a body.
    sides: BTreeMap<String, String>,
    /// The file the body being walked was written in — the document (`None`) or a module — which
    /// is what a call's component name is resolved from (`Program::resolve_component`).
    module: Option<usize>,
    /// How a predicate's body is being walked (§6.21): as itself; for what it makes and nothing
    /// it states (the geometry a tangency at a point reads its derivatives over); or a second
    /// time for a tangency, each relation emitted as its derivative and nothing made again.
    pass: apply::Pass,
}

impl Scope {
    /// The derivative a body walked for a tangency states its relations as (§6.21).
    fn twin(&self) -> Option<&crate::syntax::Along> {
        match &self.pass {
            apply::Pass::Along(a) => Some(a),
            apply::Pass::Itself | apply::Pass::Made => None,
        }
    }

    /// The innermost prefix — what a name declared here is put under.
    fn prefix(&self) -> &str {
        self.prefixes.first().map(String::as_str).unwrap_or("")
    }

    /// The innermost *instance's* prefix — what a dotted name read inside a body is made
    /// absolute under (`t.w` in `u` is `u.t.w`).  A block copy's prefix (`#3.0.`) is skipped: a
    /// `cycle` is a repetition of statements written in the body around it.
    fn instance_prefix(&self) -> &str {
        self.prefixes.iter().map(String::as_str).find(|p| !is_copy_prefix(p)).unwrap_or("")
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

/// An instance's `in PLANE`, and where it was written — see `Scope::in_plane`.
#[derive(Clone)]
struct InPlane {
    owner: String,
    plane: Ref,
    prefixes: Vec<String>,
    closed: bool,
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
    /// Every `ring` expanded, its centre resolved — what the elaborator turns copies about.
    pub rings: Vec<RingInfo>,
    /// Every set drawn as an element, for the elaborator to judge (`lowering`, #105).
    pub lowered: Vec<Lowered>,
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
}

/// A block over a chain's edges, set aside by the walk until the chain it names can be found:
/// its count is the chain's length, and the chain may stand further down or inside an instance
/// the walk has not reached (P2).  The block statement itself stands in the walk's output where
/// it was met, at its own path, and its copies replace it there.
struct Pending {
    st: Stmt,
    scope: Scope,
    vals: BTreeMap<String, Aff>,
    path: Vec<PathStep>,
    depth: usize,
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
    /// A formal bound to an actual: one name for what another names.  Resolved transitively,
    /// after the walk.
    aliases: Vec<(String, Ref, Scope)>,
    instances: Vec<InstanceInfo>,
    /// `Some` while a component is expanded over its formals as variables — see `Sym`.
    sym: Option<Sym>,
    /// Root values, used to diagnose accidental capture by a closed component.
    file_vals: BTreeMap<String, Aff>,
    /// Module values exported to importing root bodies; component bodies receive arguments.
    module_vals: Vec<Option<BTreeMap<String, Aff>>>,
    diagnostics: Vec<Diag>,
    /// Every call already read for how its arguments are written (`check_call`), by where it is
    /// written.  The question is about the *text* — which formal a written argument lands on — so
    /// it is asked once per call however many times the walk binds it: thirty copies of a `cycle`
    /// are one mistake.  A call the core wrote itself (`edit::add_rectangle`'s instance, a curve's
    /// arguments) carries no span and so is read at most once, which costs nothing: it labels
    /// every number by construction.
    called: BTreeSet<Span>,
    /// The document says `use std`, so it has the standard datums (`StandardDatums`).
    standard_datums: bool,
    group_names: BTreeSet<String>,
    group_bindings: Vec<(String, Span)>,
    group_fields: Vec<(String, Span)>,
    /// The dotted names a dimension's text reads that nothing numeric in scope answers to, with
    /// where each was written and the scope to resolve it in: judged once every name is known
    /// (`resolve`), since only then can `k.r` (a circle's radius) be told from `t.w` (a named
    /// dimension of the instance `t`).
    dim_reads: Vec<(String, Span, Scope)>,
    /// Blocks over a chain's edges, waiting for their chain — see `Pending`.
    pending: Vec<Pending>,
    /// Statements held aside in the walk's own output while a deferred block is expanded into a
    /// vector of its own, so the statement cap counts them.
    held: usize,
    /// The unknowns declared so far — see `Expansion::unknowns`.
    unknowns: BTreeMap<String, crate::model::Declared>,
    /// Where a statement read a name nothing declares (E101): the statement is not emitted, so
    /// the name never reaches the expression graph to become an unknown of its own making.
    refused: Vec<Span>,
    /// The components being expanded, outermost first: a call to one already here is a
    /// component instantiating itself (E003), refused rather than walked until the depth cap.
    instantiating: Vec<&'a Component>,
    /// The rings expanded, waiting for their centres to be resolved with everything else: the
    /// copies' prefix, the centre as written and the scope it was written in, the count, the
    /// block.
    rings: Vec<(String, Ref, Scope, usize, Span)>,
    /// Those resolved — see `Expansion::rings`.
    ring_infos: Vec<RingInfo>,
    /// Each relation word reached, read once — see `expand_word`.
    word_kinds: BTreeMap<usize, Option<std::rc::Rc<BTreeMap<String, words::Param>>>>,
    /// The relation words being applied, outermost first, each with where its outermost use is
    /// written: one reached again inside its own application is defined in terms of itself.
    applying_words: Vec<(usize, Span)>,
    /// The names an application bound to what its use wrote (`apply::bind_to_use`): one that
    /// names nothing is said once, at the operand, and not again inside the body.
    use_aliases: BTreeSet<String>,
    /// Every set the walk made, by absolute name (§6.21) — see `sets::Site`.
    sets: BTreeMap<String, sets::Site>,
    /// Those drawn as elements (`lowering`), by the same name.
    lowered: BTreeSet<String>,
    /// The sets the elaborator refused as elements, walked as sets (`expand_with`).
    unlowered: &'a BTreeSet<String>,
    /// Whether the uses of sets are being applied, so a set made now is made inside one.
    applying_sets: bool,
}

/// Expand a program's root component into a flat list of declarations, constraints, gauges and
/// orientations, with every name made absolute.
pub fn expand(prog: &Program, units: Units) -> Expansion {
    expand_with(prog, units, &BTreeSet::new())
}

/// `expand`, with the sets named in `refused` walked as sets though their bodies are elements' —
/// those the elaborator found are none (`lowering::refused`).
pub fn expand_with(prog: &Program, units: Units, refused: &BTreeSet<String>) -> Expansion {
    let mut w = Walk::new(prog, units, None);
    w.unlowered = refused;
    let root = prog.root();
    // A definitions-only file has nothing to expand. In particular, imported parameters must
    // not be evaluated in a unit system that only an eventual instantiating model will supply.
    if root.body.is_empty() {
        return w.finish();
    }
    let scope = Scope { prefixes: vec![String::new()], ..Scope::default() };
    let mut vals: BTreeMap<String, Aff> = BTreeMap::new();
    w.body(&root.body, &scope, &mut vals, &[], 0);
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
    let mut vals: BTreeMap<String, Aff> = BTreeMap::new();
    for f in &comp.formals {
        match f.ty {
            Ty::Ent(_) | Ty::Group => {
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
        let shadowed = prog.root().body.iter().any(|st| match &st.kind {
            StmtKind::Instance(i) => i.name.text == "std",
            StmtKind::Decl(d) => d.name.key().text == "std",
            StmtKind::Group(g) => g.name.text == "std",
            _ => false,
        });
        // the datums are the standard library's names like any other (§14.4): the document
        // reaches `std.front` through a `use std` of its own
        let used = prog.uses.iter().any(|u| u.name == "std");
        let standard_datums = sym.is_none() && standard.is_some() && used && !shadowed;
        Walk {
            private_names: BTreeMap::new(),
            prog,
            units,
            out: Vec::new(),
            names: if standard_datums {
                standard.unwrap().body.iter().filter_map(|st| match &st.kind {
                    StmtKind::Decl(d) => Some(format!("std.{}", d.name.key().text)),
                    _ => None,
                }).collect()
            } else { BTreeSet::new() },
            aliases: Vec::new(),
            instances: Vec::new(),
            sym,
            file_vals: BTreeMap::new(),
            module_vals: vec![None; prog.modules.len()],
            diagnostics: Vec::new(),
            called: BTreeSet::new(),
            standard_datums,
            group_names: BTreeSet::new(),
            group_bindings: Vec::new(),
            group_fields: Vec::new(),
            dim_reads: Vec::new(),
            pending: Vec::new(),
            held: 0,
            unknowns: BTreeMap::new(),
            refused: Vec::new(),
            instantiating: Vec::new(),
            rings: Vec::new(),
            ring_infos: Vec::new(),
            word_kinds: BTreeMap::new(),
            applying_words: Vec::new(),
            use_aliases: BTreeSet::new(),
            sets: BTreeMap::new(),
            lowered: BTreeSet::new(),
            unlowered: &NONE,
            applying_sets: false,
        }
    }

    fn err(&mut self, code: Code, span: Span, message: impl Into<String>) {
        self.diagnostic(Diag { code, span, stmt: None, message: message.into() });
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

    fn finish(mut self) -> Expansion {
        self.expand_pending();
        // a set's uses, once every set is known (§6.21); a use may hold a block over a chain
        self.expand_sets();
        self.expand_pending();
        let (mut flat, mut aliases) = self.resolve();
        let lowered = self.lowered(&aliases);
        if self.standard_datums {
            // A document that says `use std` has the standard datums, as a CAD part has its
            // origin planes, whether or not anything refers to them yet: the workspace offers
            // them as places to draw.  They are ordinary library statements with their own spans.
            // They follow the document's own, so its points keep the indices they would have
            // without them; resolution is by name, so nothing reads them the worse for it.
            let k = self.prog.modules.iter().position(|m| m.name == "std");
            let comp = self.prog.component_in(k, "StandardDatums").unwrap();
            let scope = Scope { prefixes: vec!["std.".into()], module: k, ..Scope::default() };
            let mut vals = self.module_params(comp.module.unwrap());
            self.body(&comp.body, &scope, &mut vals, &[], 1);
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
            rings: self.ring_infos,
            lowered,
        }
    }
}
