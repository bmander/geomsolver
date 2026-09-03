//! Elaboration: a Solvent program becomes a `Sketch`, and a `Sketch` becomes a program.
//!
//! `elaborate` is `io::from_json` with a different front end, and that is an **invariant, not a
//! coincidence**.  It walks the declarations per kind in `primitives()` order, builds through the
//! same constructors, adds every constraint with `add_quiet`, and evaluates the document's
//! expressions exactly once at the end.  Doing it in that order is what makes the parameter
//! vector of an elaborated program identical to that of a loaded document — so a drawing can be
//! saved as text, loaded, and be the same drawing, parameter for parameter.
//!
//! `to_program` goes the other way, and is the migration in one function: every `.json` document
//! ever saved becomes a program by loading it and lifting it.
//!
//! Elaboration **never returns `Err`**.  It returns whatever geometry it could build with the
//! diagnostics beside it, because a panel has to show the drawing *and* the error, and a program
//! with one bad line must still draw the other twenty.  It is the bargain `expr::evaluate`
//! already strikes — an expression that will not compute keeps its last number, and the report
//! says what is wrong.  Whether to adopt the result is the caller's to decide, from `ok()`.

use crate::constraints::{Arg as CArg, CKind, Constraint, SpecKind};
use crate::curve;
use crate::decompose;
use crate::expr;
use crate::io;
use crate::model::{EntKind, EntRef, Field, Sketch};
use crate::rng::Rng;
use crate::style::Classes;
use crate::syntax::{
    entity_name, line_col, num, Arg, AtRef, Attitude, Decl, DeclName, Kid, Name, Named,
    Program, Ref, Relation, Seg, Span, Stmt, StmtId, StmtKind,
};
use std::collections::{BTreeMap, BTreeSet};

/* -- diagnostics ------------------------------------------------------------------- */

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Note,
    Warning,
    Error,
}

/// A spec §16 code, plus the ones this implementation adds.
///
/// A *code* is what a front end can act on; a message is for a reader and may be reworded.  The
/// `E1xx` block is ours: the spec numbers the errors a language has, and these are the ones a
/// language over *this* model has as well.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Code {
    /// redeclaration within a body
    E001,
    /// an argument a formal list would silently take for another: a positional one after a
    /// labelled one, or a number given by position (§4.1)
    E004,
    /// type mismatch within an alias class
    E040,
    /// a cyclic definitional dependency: a plane folded from itself (§6.7)
    E041,
    /// a point given two planes (§6.7)
    E060,
    /// a `project` the model refuses: a point on no plane, both on one, or parallel planes
    /// (§6.7) — the core's own words, given a span
    E061,
    /// a `use` nothing resolves (§14.4)
    E070,
    /// a component defined twice, across the document and its modules (§14.4)
    E071,
    /// syntax
    E100,
    /// no such name
    E101,
    /// not a constraint type
    E102,
    /// not a shape the model can build
    E103,
    /// longer than the model will hold
    E104,
    /// `ground`/`fix` on something the document cannot express
    E105,
    /// not yet: a construct the language has and elaboration does not
    E106,
    /// an expression that would not compute — the last number stands
    W110,
    /// a free variable: which dimensions it ties together
    W111,
    /// a declaration over a built-in name (§3.3, §5): the built-in is what an expression reads
    W112,
}

impl Code {
    pub fn as_str(self) -> &'static str {
        match self {
            Code::E001 => "E001",
            Code::E004 => "E004",
            Code::E040 => "E040",
            Code::E041 => "E041",
            Code::E060 => "E060",
            Code::E061 => "E061",
            Code::E070 => "E070",
            Code::E071 => "E071",
            Code::E100 => "E100",
            Code::E101 => "E101",
            Code::E102 => "E102",
            Code::E103 => "E103",
            Code::E104 => "E104",
            Code::E105 => "E105",
            Code::E106 => "E106",
            Code::W110 => "W110",
            Code::W111 => "W111",
            Code::W112 => "W112",
        }
    }

    pub fn severity(self) -> Severity {
        match self {
            Code::W110 | Code::W111 | Code::W112 => Severity::Warning,
            _ => Severity::Error,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Diag {
    pub code: Code,
    pub span: Span,
    pub stmt: Option<StmtId>,
    pub message: String,
}

impl Diag {
    pub fn severity(&self) -> Severity {
        self.code.severity()
    }

    /// 1-based line and column, against the program the span indexes.
    pub fn at(&self, text: &str) -> (u32, u32) {
        line_col(text, self.span.lo)
    }
}

/* -- the source map ---------------------------------------------------------------- */

/// A statement is reached once per instance of every block enclosing it, outermost first.
///
/// Always empty in the flat subset, and here from the start because everything that keys on a
/// `Site` — the writeback rule, the diagnostics, a selection that has to survive — would
/// otherwise have to grow a second key later.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct InstPath(pub Vec<u32>);

/// Where something in the sketch came from.
#[derive(Clone, Debug)]
pub struct Site {
    pub stmt: StmtId,
    pub span: Span,
    pub path: InstPath,
}

/// What each thing in the sketch came from, and what each statement made.
///
/// Built by `elaborate` and thrown away with the sketch it describes: an entity is a position and
/// a constraint id is a counter, so a map that outlived one elaboration would name whatever
/// inherited its numbers.  A caller that needs an identity to *survive* holds the statement, not
/// the entity, and re-resolves it here.
#[derive(Clone, Debug, Default)]
pub struct SourceMap {
    pub of_entity: BTreeMap<EntRef, Site>,
    pub of_constraint: BTreeMap<u32, Site>,
    /// Every name an entity was declared or aliased under **that the source calls it** — what a
    /// window shows, a selection crosses a re-elaboration on, and the report publishes.  A `Vec`
    /// from the start, because a formal puts a second name on the entity it is given and costs
    /// no residual for doing it; empty for an entity the source calls nothing, which is the
    /// whole of what a reader has to ask.
    ///
    /// Deliberately *not* the same set as `by_name`, which is resolution and holds every key as
    /// well.  Which of the two a name joins is `bind`'s argument and is decided where the name
    /// is minted (issue #39); no reader re-derives it from the characters.
    ///
    /// Nor is it "a name that can be written back into source" — that is `syntax::hidden`'s
    /// narrower question, which a block prefix fails and which is why `hidden` survives.
    pub names: BTreeMap<EntRef, Vec<String>>,
    /// Of those, the entities whose name a statement may also be **written** with.  Strictly
    /// narrower than `names`, and a block's copy is the whole of the difference: `#3.0.p` is
    /// what the source calls that copy and carries a `#` no tokenizer will give back.
    writable: BTreeSet<EntRef>,
    by_name: BTreeMap<String, EntRef>,
    made: BTreeMap<StmtId, Vec<Made>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Made {
    Ent(EntRef),
    Con(u32),
    /// A gauge or a flag: it names something already in the map rather than making anything.
    Gauge,
}

impl SourceMap {
    pub fn ent_named(&self, n: &str) -> Option<EntRef> {
        self.by_name.get(n).copied()
    }

    pub fn made_by(&self, s: StmtId) -> &[Made] {
        self.made.get(&s).map(|v| v.as_slice()).unwrap_or(&[])
    }

    pub fn site_of(&self, e: EntRef) -> Option<&Site> {
        self.of_entity.get(&e)
    }

    pub fn site_of_constraint(&self, id: u32) -> Option<&Site> {
        self.of_constraint.get(&id)
    }

    /// File a name under an entity, told what kind of name it is.
    ///
    /// The three questions of `syntax::Named`, and one table each: *everything* resolves, only a
    /// name the source calls the thing is one to show, and only some of those may be written
    /// into a statement.  A reader asks the table and never the characters (issue #39).
    ///
    /// A name minted after the map was made — `edit::reconcile` splicing one into an anonymous
    /// declaration — comes through here too, and needs nothing special: the entity had no name
    /// to be preferred over, because the key it elaborated under was never filed as one.
    pub(crate) fn bind(&mut self, name: &str, e: EntRef, named: Named) {
        self.by_name.insert(name.to_string(), e);
        if named.shown() {
            self.names.entry(e).or_default().push(name.to_string());
        }
        if named.writable() {
            self.writable.insert(e);
        }
    }

    /// What the source calls an entity, where it calls it anything.
    pub fn name_of(&self, e: EntRef) -> Option<&String> {
        self.names.get(&e)?.first()
    }

    /// The same, where a statement may also be **written** with it — so `None` for one copy of a
    /// block, which the source calls `#3.0.p` and cannot say.
    pub(crate) fn writable_name(&self, e: EntRef) -> Option<&String> {
        self.writable.contains(&e).then(|| self.name_of(e)).flatten()
    }

    /// Every entity a statement made, in the order `program::build` made them — the declaration's
    /// own entity first, then the children it minted.  Two callers walk it for that reason
    /// (`edit::commit_seeds`, `edit::reconcile`), so the order is stated once, here.
    pub(crate) fn ents_made_by(&self, s: StmtId) -> impl Iterator<Item = EntRef> + '_ {
        self.made_by(s).iter().filter_map(|m| match *m {
            Made::Ent(e) => Some(e),
            _ => None,
        })
    }

    fn record(&mut self, st: &Stmt, what: Made) {
        let site = Site { stmt: st.id, span: st.span, path: InstPath::default() };
        match what {
            Made::Ent(e) => {
                self.of_entity.insert(e, site);
            }
            Made::Con(id) => {
                self.of_constraint.insert(id, site);
            }
            Made::Gauge => {}
        }
        self.made.entry(st.id).or_default().push(what);
    }
}

/// A sketch, where each of its parts came from, and what was wrong with the program.
#[derive(Default)]
pub struct Elaborated {
    pub sketch: Sketch,
    pub map: SourceMap,
    pub diags: Vec<Diag>,
    /// The program every span in here indexes.  Carried rather than borrowed: a span without the
    /// text it cuts is a pair of numbers about nothing, an edit needs the statements as well as
    /// the characters, and the caller that most needs all of it is across an ABI where a borrow
    /// cannot follow.
    pub program: Program,
    /// Whether the sketch has been moved out.  There was only ever one, and a second taker would
    /// get an empty sketch that looked like a real one.
    pub taken: bool,
}

impl Elaborated {
    /// The source every span indexes.
    pub fn text(&self) -> &str {
        self.program.text()
    }

    /// Take a new source that says the *same statements* — a splice a `Kind::Numeric` edit made,
    /// where a number changed and nothing else did.
    ///
    /// The drawing is not rebuilt: that is the whole value of the classification, since a
    /// re-elaboration is a new `Sketch` and a new `Sketch` is a lost plan, a lost compiled system
    /// and a lost selection.  But the spans **must** follow, or the next edit computed against
    /// this elaboration would splice at an offset the text no longer has.  So the source is
    /// re-*parsed* — cheap, and exact, because the same statements in the same order mint the same
    /// ids — and every site is re-stamped from the statement it names.
    ///
    /// Returns false, changing nothing, if the new text does not parse or has lost a statement
    /// this map still names; the caller then re-elaborates, which is always correct.
    pub fn retext(&mut self, text: &str) -> bool {
        let Some(prog) = reparse(text, &self.program) else { return false };
        let at = spans(&prog);
        // "the same statements" is the caller's claim; if it is false, refuse rather than quietly
        // dropping what the map knows.  The caller then elaborates, which is always correct.
        if !self.sites().all(|s| at.contains_key(&s.stmt)) {
            return false;
        }
        self.restamp(&at, Keep::All);
        self.program = prog;
        true
    }

    /// Take a source that has gained statements for things the sketch already holds, and extend
    /// the map onto them.
    ///
    /// The counterpart of `retext` for a structural splice, and it exists for the same reason:
    /// the drawing is not rebuilt.  Nothing about it changed — a gesture had already made these
    /// entities, and the source is only catching up — so re-elaborating would throw away a sketch
    /// that is already right, along with every proxy a caller is holding into it.
    ///
    /// `made` says what each appended statement was written for, in the order they were appended,
    /// which is the order they now sit in at the end of the root body.  False, changing nothing,
    /// if the new text does not parse or does not end with them.
    pub fn adopt(&mut self, text: &str, made: &[Made]) -> bool {
        let Some(prog) = reparse(text, &self.program) else { return false };
        let body = &prog.root().body;
        if body.len() < made.len() {
            return false;
        }
        let tail = &body[body.len() - made.len()..];
        // a statement may have gone as well as arrived, and one that went takes its entries with it
        self.restamp(&spans(&prog), Keep::Live);
        for (st, m) in tail.iter().zip(made) {
            let site = Site { stmt: st.id, span: st.span, path: InstPath::default() };
            match *m {
                Made::Ent(r) => {
                    if let StmtKind::Decl(d) = &st.kind {
                        self.map.bind(&d.name.key().text, r, d.name.named());
                    }
                    self.map.of_entity.insert(r, site);
                }
                Made::Con(id) => {
                    self.map.of_constraint.insert(id, site);
                }
                Made::Gauge => {}
            }
            self.map.made.entry(st.id).or_default().push(*m);
        }
        self.program = prog;
        true
    }

    /// Every site in the map: what the drawing was made from, entities and constraints alike.
    fn sites(&self) -> impl Iterator<Item = &Site> {
        self.map.of_entity.values().chain(self.map.of_constraint.values())
    }

    /// Point every site at where its statement now is.  The one mechanism `retext` and `adopt`
    /// share: both take a source the core spliced and have to bring the map onto it.
    fn restamp(&mut self, at: &BTreeMap<StmtId, Span>, keep: Keep) {
        if keep == Keep::Live {
            self.map.of_entity.retain(|_, s| at.contains_key(&s.stmt));
            self.map.of_constraint.retain(|_, s| at.contains_key(&s.stmt));
        }
        for s in self.map.of_entity.values_mut().chain(self.map.of_constraint.values_mut()) {
            if let Some(&sp) = at.get(&s.stmt) {
                s.span = sp;
            }
        }
    }

    /// Whether the program said anything the elaborator could not honour.  A warning does not
    /// count: an expression that will not compute is a thing to report, not a thing to refuse.
    pub fn ok(&self) -> bool {
        !self.diags.iter().any(|d| d.severity() == Severity::Error)
    }

    pub fn errors(&self) -> impl Iterator<Item = &Diag> {
        self.diags.iter().filter(|d| d.severity() == Severity::Error)
    }
}

/* -- elaboration ------------------------------------------------------------------- */

/// Names bound to entities before anything is built, so forward reference works — which spec P2
/// requires, since a body is a set and a set has no "before".
///
/// The union-find is a singleton per class in the flat subset and is here from the start: an
/// alias class *is* how a formal costs nothing, so aliasing is a property of this pass rather
/// than a later change to the model.
#[derive(Default)]
struct Resolver {
    of: BTreeMap<String, EntRef>,
    declared_at: BTreeMap<String, Span>,
    /// Each declaration's child slots, as the names it wrote — what `follow_building` reads
    /// where the entity itself is not built yet (build order is per kind, so a child slot that
    /// reaches into an entity of a later kind — `line t(p3, k.start)` with `k` an arc — has
    /// only the declaration to ask).  `None` where a slot holds a seed or nothing, since the
    /// point it mints does not exist until the parent is built.
    kids: BTreeMap<String, Vec<Option<Ref>>>,
}

impl Resolver {
    fn lookup(&self, r: &Ref) -> Option<EntRef> {
        self.of.get(&r.root.text).copied()
    }

    /// The name declaration `name` (of kind `kind`) wrote in its child field `f`, mirroring
    /// `follow`'s reading of the built entity — the same fields, the same refusals.
    fn kid(&self, name: &str, kind: EntKind, f: &str) -> Result<Option<Ref>, String> {
        let slots = self.kids.get(name).ok_or_else(|| format!("no such entity: `{name}`"))?;
        let mut at = 0usize;
        for (n, k) in kind.fields() {
            match k {
                Field::Scalar => {}
                Field::Child => {
                    if *n == f {
                        return Ok(slots.get(at).cloned().flatten());
                    }
                    at += 1;
                }
                Field::List => {
                    if *n == f {
                        return Err(format!("`{f}` is a list, so it needs an index"));
                    }
                    at += 1;
                }
            }
        }
        let named: Vec<&str> =
            kind.fields().iter().filter(|(_, k)| *k == Field::Child).map(|(n, _)| *n).collect();
        Err(if named.is_empty() {
            format!("a {} has no parts", kind.as_str())
        } else {
            format!("a {} has {}, not `{f}`", kind.as_str(), named.join(", "))
        })
    }
}

/// `follow`, for the one walk that runs while the sketch is still being built.
///
/// Phase 2 builds per kind, so a child slot's dotted reference may reach an entity that exists
/// in name only — `line t(p3, k.start)` with `k` an arc, which builds after every line.  Where
/// the entity is built this is `follow`; where it is not, the slot's name is read off the
/// *declaration* (`Resolver::kid`) and the walk continues from what that names.  A slot that
/// holds a seed or nothing mints its point only when the parent is built, so there is nothing
/// to reach and the reference is refused rather than guessed.
fn follow_building(sk: &Sketch, res: &Resolver, e: EntRef, r: &Ref) -> Result<EntRef, String> {
    let mut e = e;
    let mut name = r.root.text.clone();
    let mut path: Vec<Seg> = r.path.clone();
    // two declarations naming their parts through each other would walk forever; the cap is
    // far past any real document, so hitting it is the cycle
    for _ in 0..64 {
        if path.is_empty() || e.i() < sk.count(e.kind) {
            return follow(sk, e, &path);
        }
        let Seg::Field(f) = &path[0] else {
            return Err("an index names a copy, not a part".to_string());
        };
        let Some(kid) = res.kid(&name, e.kind, &f.text)? else {
            return Err(format!(
                "`{name}` does not name its {}, and `{name}` is not built yet: name the point \
                 itself",
                f.text
            ));
        };
        let Some(ne) = res.lookup(&kid) else {
            return Err(format!("no such entity: `{}`", kid.root.text));
        };
        e = ne;
        name = kid.root.text.clone();
        path = kid.path.iter().chain(path[1..].iter()).cloned().collect();
    }
    Err(format!("`{}` names its parts in a circle", r.root.text))
}

/// Follow a reference's field path to the sub-entity it names — `root.center` is the circle's
/// centre point, and `a0.start` an arc's.
///
/// A `Scalar` field is not an entity and is not followed here: `c0.r` is a *number*, and the one
/// statement that names one is `fix`, which reads the path itself.
fn follow(sk: &Sketch, mut e: EntRef, path: &[Seg]) -> Result<EntRef, String> {
    for seg in path {
        let Seg::Field(f) = seg else {
            return Err("an index names a copy, not a part".to_string());
        };
        let fields = e.kind.fields();
        let kids = sk.children(e);
        let mut at = 0usize;
        let mut found = None;
        for (name, kind) in fields {
            match kind {
                Field::Scalar => {}
                Field::Child => {
                    if *name == f.text {
                        found = kids.get(at).copied();
                    }
                    at += 1;
                }
                Field::List => {
                    if *name == f.text {
                        return Err(format!("`{}` is a list, so it needs an index", f.text));
                    }
                }
            }
        }
        match found {
            Some(k) => e = k,
            None => {
                let named: Vec<&str> = fields
                    .iter()
                    .filter(|(_, k)| *k == Field::Child)
                    .map(|(n, _)| *n)
                    .collect();
                return Err(if named.is_empty() {
                    format!("a {} has no parts", e.kind.as_str())
                } else {
                    format!("a {} has {}, not `{}`", e.kind.as_str(), named.join(", "), f.text)
                });
            }
        }
    }
    Ok(e)
}

/// What to do with a site whose statement is no longer in the program.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Keep {
    /// Every site's statement is expected to still be there; the caller has already checked.
    All,
    /// A statement may have been deleted, and its sites go with it.
    Live,
}

/// Parse a source that is expected to be good.  `None` when it is not: both callers are taking a
/// text the core itself just spliced, so a parse error means the splice was wrong, and the answer
/// is to refuse and let the caller elaborate from scratch.
fn reparse(text: &str, like: &Program) -> Option<Program> {
    let (mut prog, errs) = crate::syntax::parse(text);
    // the same modules, from the texts already in hand: a re-parse asks the host nothing
    let linked = crate::modules::relink(&mut prog, like);
    (errs.is_empty() && linked.is_empty()).then_some(prog)
}


/// Where every statement in a program sits, blocks included — a statement inside one is still
/// reached by its id.
fn spans(prog: &Program) -> BTreeMap<StmtId, Span> {
    fn stamp(st: &Stmt, out: &mut BTreeMap<StmtId, Span>) {
        out.insert(st.id, st.span);
        if let StmtKind::Block(b) = &st.kind {
            for inner in b.stmts() {
                stamp(inner, out);
            }
        }
    }
    let mut out = BTreeMap::new();
    for c in prog.components.iter() {
        for st in c.body.iter() {
            stamp(st, &mut out);
        }
    }
    out
}

/// **A declaration over a built-in name is said** (issue #48, item 2; §3.3, §5).
///
/// `param tau = 35deg` gave one name two numbers.  A *text* carrying `tau` is substituted by the
/// flattener and read 35°; a number *worked out* by `expr::eval` read a full turn, the constants
/// being known before the document is — so the lever the angle turned stood at 360° and nothing
/// was said anywhere.  A named dimension of a built-in name is already refused where it is parsed
/// (`expr::parse_in`); this is the other three ways a document declares a number, said at the
/// **declaration**, which is the place a reader can act on and the one place that is written once
/// however many times the name is read.
///
/// Warned and not refused, because the drawing is not wrong — the *name* is, and a document that
/// has one wants renaming rather than rejecting.  It is a fact about the text, so it is asked of
/// the text: every component the program holds, instantiated or not, and every module's own body,
/// each declaration once.
fn shadowing(p: &Program, diags: &mut Vec<Diag>) {
    fn say(name: &Name, what: &str, stmt: Option<StmtId>, diags: &mut Vec<Diag>) {
        let Some(kind) = expr::builtin(&name.text) else { return };
        diags.push(Diag {
            code: Code::W112,
            span: name.span,
            stmt,
            message: format!(
                "`{}` is {kind}, and {what} of that name does not shadow it — an expression \
                 reads the built-in wherever a number is worked out (§3.3), so this name is \
                 read two ways.  Rename it.",
                name.text
            ),
        });
    }
    fn body(stmts: &[Stmt], diags: &mut Vec<Diag>) {
        for st in stmts {
            match &st.kind {
                StmtKind::Param(d) => say(&d.name, "a `param`", Some(st.id), diags),
                StmtKind::Block(b) => {
                    if let Some(i) = &b.binder {
                        say(i, "a block's index", Some(st.id), diags);
                    }
                    body(&b.body, diags);
                }
                _ => {}
            }
        }
    }
    // the root stands among the components, and a module's components with it; what a module's
    // own body adds is its top-level params, which its components read (§6.3)
    let mut said: Vec<Diag> = Vec::new();
    for c in &p.components {
        for f in &c.formals {
            say(&f.name, "a formal", None, &mut said);
        }
        body(&c.body, &mut said);
    }
    for m in &p.modules {
        body(&m.root.body, &mut said);
    }
    // in the order a reader meets them: the components come out of the program in link order,
    // and a document is read down the page
    said.sort_by_key(|d| d.span.lo);
    diags.append(&mut said);
}

pub fn elaborate(p: &Program) -> Elaborated {
    let mut diags: Vec<Diag> = Vec::new();
    let mut map = SourceMap::default();
    let mut sk = Sketch::new();

    // -- phase 0: the document's unit, before *anything* reads a number.
    //
    // A literal with a unit on it converts to this one, and a document that names none is in
    // drawing units (spec §3.3).  It is read from the **unexpanded root** because a unit is
    // document preamble and a component is reusable: a `unit` line anywhere else is refused
    // below rather than quietly doing nothing.  It comes before the curve families, because a
    // family's body is a number-bearing text like any other.
    let mut said = false;
    for st in &p.root().body {
        let StmtKind::Unit(n) = &st.kind else { continue };
        if said {
            diags.push(Diag {
                code: Code::E040,
                span: n.span,
                stmt: Some(st.id),
                message: "the document's unit is already stated above — one document, one \
                          unit"
                    .to_string(),
            });
            continue;
        }
        said = true;
        match crate::units::Units::with_length(&n.text) {
            Ok(u) => sk.units = u,
            Err(message) => {
                diags.push(Diag { code: Code::E040, span: n.span, stmt: Some(st.id), message })
            }
        }
    }

    // -- a name a document declares over a built-in is said, before anything reads either.
    shadowing(p, &mut diags);

    // -- phase 1: names, in one pre-pass.  Indices come from declaration order within a kind,
    // which is `primitives()` order, which is the order phase 2 builds in.
    let expansion = crate::flatten::expand(p, sk.units);
    for (code, span, message) in &expansion.coded {
        diags.push(Diag { code: *code, span: *span, stmt: None, message: message.clone() });
    }
    for (span, message) in &expansion.errors {
        diags.push(Diag {
            code: if message.starts_with("no such") {
                Code::E101
            } else if message.ends_with("is declared twice") {
                Code::E001
            } else {
                Code::E103
            },
            span: *span,
            stmt: None,
            message: message.clone(),
        });
    }
    let mut res = Resolver::default();
    let mut count: BTreeMap<EntKind, u32> = BTreeMap::new();
    // a redeclaration is skipped rather than merged, and *which* statement was skipped is
    // remembered here: inferring it later from the name would find the one that won
    let mut skip: BTreeSet<StmtId> = BTreeSet::new();
    let body: Vec<&Stmt> = expansion.flat.iter().map(|f| &f.stmt).collect();
    // the style sheet, before anything is built: it says nothing about what the drawing is, so
    // it is collected once and never consulted again by anything here (spec §14)
    let mut sheet = crate::style::Sheet::new();
    for st in &body {
        // a `unit` inside a component or a block is expanded into the flat list and read by
        // nobody — phase 0 takes the root's alone, a component being reusable and a unit being
        // the document's.  Silence there is exactly what §13.1 forbids, so it is a diagnostic.
        if let StmtKind::Unit(n) = &st.kind {
            if !p.root().body.iter().any(|r| r.id == st.id) {
                diags.push(Diag {
                    code: Code::E040,
                    span: n.span,
                    stmt: Some(st.id),
                    message: "a document's unit is stated once, at the top — not inside a \
                              component or a block"
                        .to_string(),
                });
            }
        }
        if let StmtKind::Style(r) = &st.kind {
            // a class stated twice cascades, later over earlier — the same rule that decides a
            // conflicting property between two classes on one declaration
            sheet.entry(r.name.text.clone()).or_default().over(&r.style);
        }
    }
    sk.set_sheet(sheet);
    for st in &body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        // the *key*, which is resolution's question: an anonymous key is its own offset and a
        // copy's carries its prefix, so only a name the source wrote twice can actually collide
        let key = &d.name.key().text;
        if let Some(&was) = res.declared_at.get(key) {
            // …but the message shows what the source calls it, and spells the kind where it
            // calls it nothing, so a key cannot leak here even if that argument ever breaks
            let who = d.name.shown().map_or_else(
                || crate::syntax::decl_head(d.kind, &d.name),
                |n| n.text.clone(),
            );
            diags.push(Diag {
                code: Code::E001,
                span: d.name.span(),
                stmt: Some(st.id),
                message: format!(
                    "`{who}` is declared twice; the first is at line {}",
                    p.line_col(was.lo as usize).0
                ),
            });
            skip.insert(st.id);
            continue; // the second, so every later reference still resolves to the first
        }
        let n = count.entry(d.kind).or_insert(0);
        res.of.insert(key.clone(), EntRef::new(d.kind, *n as usize));
        res.declared_at.insert(key.clone(), d.name.span());
        res.kids.insert(
            key.clone(),
            d.children
                .iter()
                .map(|g| match g.first() {
                    Some(crate::syntax::Kid::Ref(r)) if g.len() == 1 => Some(r.clone()),
                    _ => None,
                })
                .collect(),
        );
        *n += 1;
    }

    // every plane's attitude, before any plane is built: a plane folded from another needs
    // the parent's basis, and the build itself must stay in body order (phase 1 assigned the
    // indices in it), so the arithmetic is done here, memoised, and handed to the build
    let bases = plane_bases(&body, &res, &skip, sk.units, &mut diags);

    // -- phase 2: geometry, per kind in `primitives()` order.  The same walk `io::from_json`
    // makes, through the same constructors, so the two produce the same parameter vector.
    let mut built: BTreeMap<EntRef, bool> = BTreeMap::new();
    // seeds that read geometry, settled once every declaration has a seed of its own
    let mut deferred: Vec<Deferred> = Vec::new();
    // `primitives()` order, and **curves last**: a curve is written over other entities, so
    // every kind it may name has to exist before it does.  The same reason `io::graft` grafts
    // them last.
    for kind in [
        EntKind::Point,
        EntKind::Line,
        EntKind::Circle,
        EntKind::Arc,
        EntKind::Spline,
        EntKind::Plane,
        EntKind::Curve,
    ] {
        for st in &body {
            let StmtKind::Decl(d) = &st.kind else { continue };
            if d.kind != kind || skip.contains(&st.id) {
                continue;
            }
            let mut anon: Vec<(String, EntRef)> = Vec::new();
            match build(
                &mut sk,
                &res,
                d,
                st,
                &bases,
                &mut diags,
                &mut anon,
                &mut deferred,
                p,
                &expansion.instances,
            ) {
                Some(e) => {
                    built.insert(e, true);
                    map.bind(&d.name.key().text, e, d.name.named());
                    map.record(st, Made::Ent(e));
                    // an anonymous child's name *is* its dotted path, so it is bound like any
                    // other: that is what lets a dimension name it, a selection survive a
                    // re-elaboration, and a drag of it find the slot it came from.  Whether it
                    // is a name anyone may *say* is the declaration's answer, not its own —
                    // `l.p1` under a named line, nothing under an anonymous one
                    for (name, k) in anon {
                        built.insert(k, true);
                        map.bind(&name, k, d.name.named());
                        map.record(st, Made::Ent(k));
                    }
                }
                None => {
                    // a declaration that could not be built leaves its name unbound, so every
                    // reference to it is reported where the reference is — and made nothing,
                    // so every later entity of its kind sits one index below where phase 1
                    // put it: the resolver is shifted with them, or a reference to a later arc
                    // would read the one after it (or past the end of the list)
                    if let Some(gone) = res.of.remove(&d.name.key().text) {
                        for e in res.of.values_mut() {
                            if e.kind == gone.kind && e.idx > gone.idx {
                                e.idx -= 1;
                            }
                        }
                    }
                }
            }
        }
    }

    // seeds named by geometry, once every entity has a seed to be read: in statement order, so
    // a seed that reads a seed read from a third is settled after both (§6.4)
    settle_deferred(&mut sk, &res, &deferred, &mut diags);

    // memberships, once every kind is built and before any constraint reads one: `point a in
    // top` names a plane built after the point, and `project` infers its planes from these
    memberships(&mut sk, &res, &map, &body, &skip, &mut diags);

    // -- phase 3: constraints, in statement order
    for st in &body {
        let StmtKind::Relation(r) = &st.kind else { continue };
        if let Some(id) = constrain(&mut sk, &res, r, st, &mut diags) {
            map.record(st, Made::Con(id));
            if let Some(place) = r.place {
                sk.placements.insert(id, place);
            }
        }
    }

    // -- phase 4: every expression against the whole document, once.  Per-statement evaluation
    // would be quadratic in the expression count and would make a dimension whose definition is
    // further down the file briefly a free variable — allocating an unknown the next pass retires.
    for item in expr::evaluate(&mut sk) {
        let span = map.site_of_constraint(item.id).map(|s| s.span).unwrap_or_default();
        let stmt = map.site_of_constraint(item.id).map(|s| s.stmt);
        if let Some(err) = &item.error {
            // what the fault *is* decides what is said (#43.11): a number that is not what its
            // slot takes is the E103 every `param` already gets — §3.3 names `distance(45deg)`
            // as an error — and a claim binding a free name is the E040 §9.7 promises;
            // only an expression that would not compute is a warning, since the last number
            // stands and the drawing goes on
            let (code, tail) = match err.fault {
                expr::Fault::Dimension => (Code::E103, ""),
                expr::Fault::ClaimFree => (Code::E040, ""),
                expr::Fault::Uncomputable => (Code::W110, " — the last number stands"),
            };
            diags.push(Diag {
                code,
                span,
                stmt,
                message: format!("`{}`: {err}{tail}", item.text),
            });
        } else if !item.free.is_empty() {
            diags.push(Diag {
                code: Code::W111,
                span,
                stmt,
                message: format!(
                    "`{}` is a free variable: the solver answers for it",
                    item.free.join("`, `")
                ),
            });
        }
    }

    // -- phase 5: a root choice under a key no triple of points spells, kept verbatim
    for st in &body {
        if let StmtKind::Branch(b) = &st.kind {
            sk.branches.insert(b.key.clone(), b.value);
        }
    }

    crate::modules::localize(p, &mut diags);
    Elaborated { sketch: sk, map, diags, program: p.clone(), taken: false }
}

/// A curve, compiled: a component's point over one of its numeric formals (§6.5).
///
/// The component is expanded **over its formals** (`flatten::expand_component`), so `param`s,
/// nested instances and repetition are done there and not again here, and what arrives is a
/// flat body under an empty prefix.  The variable table is the swept formal, then every
/// coordinate the entity formals contribute *in `entity_params` order* (`EntKind::scalar_names`),
/// then the other numbers the component takes.  That order is the kernel's column order, which
/// is what makes a tape's gradient a row of the Jacobian.  A computed point (`point p = (x, y)`)
/// compiles to its two tapes; any other point is a trace over the body's statements.
fn compile_curve(
    prog: &Program,
    comp: &crate::syntax::Component,
    swept: &str,
    point: &str,
    units: crate::units::Units,
) -> Result<crate::model::CurveDef, (Span, String)> {
    use crate::syntax::Ty;
    let cname = comp.name.as_ref().map(|n| n.text.clone()).unwrap_or_default();
    let mut vars = vec![swept.to_string()];
    let mut formals = Vec::new();
    let mut values = Vec::new();
    for fo in &comp.formals {
        match fo.ty {
            Ty::Ent(k) => {
                let names = k.scalar_names(&fo.name.text).ok_or_else(|| {
                    (
                        fo.span,
                        format!(
                            "a curve cannot be written over a {}: it has no fixed number of \
                             coordinates",
                            k.as_str()
                        ),
                    )
                })?;
                vars.extend(names);
                formals.push((fo.name.text.clone(), k));
            }
            _ if fo.name.text != swept => values.push(fo.name.text.clone()),
            _ => {}
        }
    }
    vars.extend(values.iter().cloned());
    let ex = crate::flatten::expand_component(prog, comp, units);
    if let Some((span, m)) = ex.errors.first() {
        return Err((*span, m.clone()));
    }
    if let Some((_, span, m)) = ex.coded.iter().find(|(c, ..)| c.severity() == Severity::Error) {
        return Err((*span, m.clone()));
    }
    let traced = ex.aliases.get(point).cloned().unwrap_or_else(|| point.to_string());
    if formals.iter().any(|(n, _)| *n == traced) {
        return Err((
            comp.span,
            format!(
                "`{point}` is geometry `{cname}` is written over, and does not move with `{swept}`"
            ),
        ));
    }
    let body: Vec<&Stmt> = ex.flat.iter().map(|f| &f.stmt).collect();
    // a computed point is the whole of what a component may say about it: nothing on the
    // sheet or in a block can hold a point to a formula beside placed geometry, so the body
    // is the one computed point — decided by shape, not by which statement is found first
    let computed = body.iter().find_map(|st| match &st.kind {
        StmtKind::Decl(d) if d.name.key().text == traced => d.computed.as_ref(),
        _ => None,
    });
    let (body, pose_of) = match computed {
        Some([(x, xspan), (y, yspan)]) => {
            if body.len() != 1 {
                return Err((
                    comp.span,
                    format!(
                        "`{point}` is a computed point, so `{cname}` may hold nothing else: a \
                         formula and placed geometry cannot both say where a point is"
                    ),
                ));
            }
            let tape = |text: &str, span: Span| -> Result<crate::tape::Tape, (Span, String)> {
                let ast = crate::expr::parse_in(text, units).map_err(|e| (span, e))?;
                crate::tape::Tape::compile(&ast.body, &vars).map_err(|e| (span, e))
            };
            let body = crate::model::CurveBody::Exprs { x: tape(x, *xspan)?, y: tape(y, *yspan)? };
            (body, Vec::new())
        }
        None => {
            let (locus, pose_of) =
                compile_trace(comp.span, &traced, &body, &vars, &formals, values.len(), units)?;
            (crate::model::CurveBody::Trace(locus), pose_of)
        }
    };
    Ok(crate::model::CurveDef {
        name: crate::model::CurveDef::key(&cname, point, swept),
        component: cname,
        port: point.to_string(),
        formals,
        values,
        param: swept.to_string(),
        vars,
        body,
        pose_of,
    })
}

/// A trace block, lowered.
///
/// The block is elaborated into a *scratch sketch* — the formals materialised as default-shaped
/// entities, the block's declarations built beside them — and each constraint is then read back
/// out through `Constraint::params_on`, the same column-mapping every compiled system uses, so
/// there is no second copy of which coordinates a kernel reads.  What leaves here is pure data:
/// rows over one variable table, ready to ride in a contact's constants (`locus`).
///
/// A dimension whose value is written over `u` and the geometry is carried by its *free twin*
/// kernel: the value becomes a derived variable `w`, defined by a tape, read as the twin's last
/// column with `(m, c)` the unit conversion — so `∂r/∂u` comes from the kernel and the tape and
/// nowhere else.
#[allow(clippy::too_many_arguments)]
fn compile_trace(
    span: Span,
    point: &str,
    body: &[&Stmt],
    vars: &[String],
    formals: &[(String, EntKind)],
    n_values: usize,
    units: crate::units::Units,
) -> Result<(crate::locus::Locus, Vec<(String, usize)>), (Span, String)> {
    use crate::locus::{Locus, Pred, Row};
    use crate::tape::Tape;
    let mut sk = Sketch::new();
    // the scratch sketch the block is lowered through reads numbers, so it is in the document's
    // units too — `at_seed` asks it what a bearing's literal is written in
    sk.units = units;
    let mut scope: BTreeMap<String, EntRef> = BTreeMap::new();
    // scratch parameter index -> variable-table slot
    let mut slot: BTreeMap<u32, usize> = BTreeMap::new();
    let mut next = 1usize; // slot 0 is the parameter
    for (name, kind) in formals {
        let e = match kind {
            EntKind::Point => EntRef::point(sk.point(0.0, 0.0, false, name)),
            EntKind::Line => {
                let a = sk.point(0.0, 0.0, false, name);
                let b = sk.point(1.0, 0.0, false, name);
                EntRef::line(sk.line(a, b))
            }
            EntKind::Circle => {
                let c = sk.point(0.0, 0.0, false, name);
                EntRef::circle(sk.circle(c, 1.0, name))
            }
            // a datum: its attitude is no column of the curve, so the page's will do
            EntKind::Plane => {
                let o = sk.point(0.0, 0.0, false, name);
                let t = sk.point(1.0, 0.0, false, name);
                EntRef::plane(sk.plane(o, t, crate::plane::Basis::page(), name))
            }
            other => {
                return Err((
                    span,
                    format!("a trace cannot yet be written over a {}", other.as_str()),
                ))
            }
        };
        for p in sk.entity_params(e) {
            slot.insert(p, next);
            next += 1;
        }
        scope.insert(name.clone(), e);
    }
    let n_theta = next - 1;
    let n_outer = vars.len();
    debug_assert_eq!(n_outer, 1 + n_theta + n_values, "variable table shape");
    let tape = |text: &str, span: Span| -> Result<Tape, (Span, String)> {
        let ast = crate::expr::parse_in(text, units).map_err(|e| (span, e))?;
        Tape::compile(&ast.body, vars).map_err(|e| (span, e))
    };
    let constant = |v: f64| -> Tape {
        Tape::compile(&crate::expr::Ast::Num(v, crate::units::Dim::SCALAR), vars)
            .expect("a number always compiles")
    };

    // -- pass 1: declarations, so a statement may read a point declared after it --------
    let mut n_q = 0usize;
    let mut seeds: Vec<Tape> = Vec::new();
    // each inner unknown's owner, so a drawn instance's pose can be read off the sheet
    let mut pose_of: Vec<(String, usize)> = Vec::new();
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        // a computed point that is not the one traced is its formula and has no unknowns here
        if d.computed.is_some() {
            continue;
        }
        if d.seed_at.is_some() && d.kind != EntKind::Point {
            return Err((st.span, "only a point takes a geometric seed".to_string()));
        }
        let seed_tape = |i: usize| -> Result<Tape, (Span, String)> {
            match d.seed_text.get(i).and_then(|t| t.as_ref()) {
                Some(t) => tape(t, *d.seed_spans.get(i).unwrap_or(&st.span)),
                None => Ok(constant(d.seed.get(i).copied().unwrap_or(0.0))),
            }
        };
        let child = |sk: &Sketch, scope: &BTreeMap<String, EntRef>, g: usize|
            -> Result<EntRef, (Span, String)>
        {
            let r = d
                .children
                .get(g)
                .and_then(|v| v.first())
                .and_then(|k| k.as_ref())
                // an anonymous declaration's key is the elaboration's, not the writer's, so the
                // message spells the kind instead — `decl_head`, the one wording both the
                // parser's errors and these diagnostics use
                .ok_or_else(|| {
                    let who = crate::syntax::decl_head(d.kind, &d.name);
                    (st.span, format!("`{who}` needs its points named"))
                })?;
            let e = scope
                .get(&r.root.text)
                .copied()
                .ok_or((r.span, format!("no such entity: `{}`", r.root.text)))?;
            follow(sk, e, &r.path).map_err(|m| (r.span, m))
        };
        let e = match d.kind {
            EntKind::Point => {
                let (sx, sy) = match &d.seed_at {
                    Some(a) => at_seed(&sk, &scope, &slot, vars, &seeds, n_outer, a, st.span)?,
                    None => (seed_tape(0)?, seed_tape(1)?),
                };
                // a scratch sketch nobody's DOF dialog reads, so the key is a fine label here
                let e = EntRef::point(sk.point(0.0, 0.0, false, &d.name.key().text));
                seeds.push(sx);
                seeds.push(sy);
                for (j, p) in sk.own_params(e).into_iter().enumerate() {
                    slot.insert(p, n_outer + n_q);
                    pose_of.push((d.name.key().text.clone(), j));
                    n_q += 1;
                }
                e
            }
            EntKind::Line => {
                let a = child(&sk, &scope, 0)?;
                let b = child(&sk, &scope, 1)?;
                if a.kind != EntKind::Point || b.kind != EntKind::Point {
                    return Err((st.span, "a line runs between points".to_string()));
                }
                EntRef::line(sk.line(a.i(), b.i()))
            }
            EntKind::Circle => {
                let c = child(&sk, &scope, 0)?;
                if c.kind != EntKind::Point {
                    return Err((st.span, "a circle's centre is a point".to_string()));
                }
                let sr = seed_tape(0)?;
                let e = EntRef::circle(sk.circle(c.i(), 0.0, &d.name.key().text));
                seeds.push(sr);
                let (j, p) = (0usize, sk.own_params(e)[0]);
                slot.insert(p, n_outer + n_q);
                pose_of.push((d.name.key().text.clone(), j));
                n_q += 1;
                e
            }
            other => {
                return Err((
                    st.span,
                    format!("a trace block cannot yet draw a {}", other.as_str()),
                ))
            }
        };
        if scope.insert(d.name.key().text.clone(), e).is_some() {
            // two anonymous declarations are two offsets, so only a written name can collide —
            // but the message asks `shown` all the same, so a key cannot leak into it
            let who = d.name.shown().map_or_else(
                || crate::syntax::decl_head(d.kind, &d.name),
                |n| n.text.clone(),
            );
            return Err((st.span, format!("`{who}` is declared twice")));
        }
    }

    // -- pass 2: constraints and orientations, lowered to rows and predicates ----------
    let mut w: Vec<Tape> = Vec::new();
    let mut rows: Vec<Row> = Vec::new();
    let mut preds: Vec<Pred> = Vec::new();
    for st in body {
        let r = match &st.kind {
            StmtKind::Decl(_) => continue,
            StmtKind::Relation(r) => r,
            _ => {
                return Err((
                    st.span,
                    "a traced component holds declarations and constraints only".to_string(),
                ))
            }
        };
        // the operator, settled against what the *block* declares — the same two lookups the
        // document's statements go through, over a scope of the block's own
        let settled = match &r.poly {
            None => None,
            Some(w) => Some(
                settle(w, &|rf| {
                    scope.get(&rf.root.text).copied().and_then(|e| follow(&sk, e, &rf.path).ok())
                        .map(|e| e.kind)
                })?,
            ),
        };
        let owned;
        let r: &Relation = match settled {
            Some((k, a)) => {
                owned = Relation { kind: k, args: a, ..r.clone() };
                &owned
            }
            None => r,
        };
        // `ccw(a, b, x)` — no residual: it selects among the discrete solution components,
        // which is what a branch is, and it is how a block says one *as a fact* rather than
        // as a place to start looking
        if matches!(r.kind, CKind::Ccw | CKind::Cw) {
            let pts: Vec<&Ref> = r
                .args
                .iter()
                .filter_map(|a| match a {
                    Some(Arg::Ref(rf)) => Some(rf),
                    _ => None,
                })
                .collect();
            if pts.len() != 3 {
                return Err((st.span, "an orientation names three points".to_string()));
            }
            let mut cols = [0u32; 6];
            for (k, rf) in pts.iter().enumerate() {
                let e = scope
                    .get(&rf.root.text)
                    .copied()
                    .ok_or((rf.span, format!("no such entity: `{}`", rf.root.text)))?;
                let e = follow(&sk, e, &rf.path).map_err(|m| (rf.span, m))?;
                if e.kind != EntKind::Point {
                    return Err((rf.span, "an orientation is about points".to_string()));
                }
                let ps = sk.point_params(e.i());
                for (j, &p) in ps.iter().enumerate() {
                    let Some(&s) = slot.get(&p) else {
                        return Err((st.span, "trace lowering lost a column".to_string()));
                    };
                    cols[2 * k + j] = s as u32;
                }
            }
            // the placed point is the one a violated predicate reflects, so it must be one
            // the block actually places
            if (cols[4] as usize) < n_outer {
                return Err((
                    pts[2].span,
                    "the third point must be one the block places".to_string(),
                ));
            }
            preds.push(Pred { ccw: r.kind == CKind::Ccw, cols });
            continue;
        }
        let spec = r.kind.spec();
        if r.kind.gauge() {
            return Err((
                st.span,
                format!(
                    "`{}` cannot appear in a trace block: a traced component holds declarations \
                     and constraints only, and its unknowns are what it places",
                    crate::syntax::snake(r.kind.name())
                ),
            ));
        }
        if r.kind == CKind::DragTarget || spec.iter().any(|(_, k)| k.is_param()) {
            return Err((
                st.span,
                format!("{} cannot appear in a trace block", r.kind.name()),
            ));
        }
        let mut cargs: Vec<CArg> = Vec::with_capacity(spec.len());
        let mut dim: Option<(SpecKind, Tape)> = None;
        for (i, (name, kind)) in spec.iter().enumerate() {
            let given = r.args.get(i).and_then(|a| a.as_ref());
            match (kind, given) {
                (k, Some(Arg::Ref(rf))) if k.is_entity() => {
                    let found = scope.get(&rf.root.text).copied();
                    cargs.push(ent_arg(&sk, found, *k, rf).map_err(|m| (rf.span, m))?);
                }
                (k, Some(Arg::Dim { text, span })) if k.is_dimension() => {
                    dim = Some((*k, tape(text, *span)?));
                    cargs.push(CArg::Num(0.0));
                }
                // a slot the core would read off the geometry (a tangency's side or sense) is
                // required too: there is no drawn geometry here to read it from, and a default
                // would silently pick the branch
                (k, None) if k.is_dimension() || k.is_entity() || r.kind.infers_arg(i) => {
                    return Err((
                        st.span,
                        format!("`{name}` must be stated: a trace block infers nothing"),
                    ));
                }
                (_, None) => cargs.push(r.kind.default_arg(i)),
                (k, Some(a)) => match scalar_arg(*k, a) {
                    Some(v) => cargs.push(v),
                    None => {
                        return Err((
                            st.span,
                            format!("`{name}`: a {} is wanted here, not {a:?}", k.as_str()),
                        ))
                    }
                },
            }
        }
        let c = Constraint::new(r.kind, cargs);
        let mut cols: Vec<u32> = Vec::new();
        for p in c.params(&sk) {
            let Some(&s) = slot.get(&p) else {
                return Err((st.span, "trace lowering lost a column".to_string()));
            };
            cols.push(s as u32);
        }
        let kid = match dim {
            Some((k, t)) => {
                let twin = r.kind.free_kernel().ok_or((
                    st.span,
                    format!("a {} cannot be stated over `u` here", r.kind.name()),
                ))?;
                // every declaration was consumed in pass 1, so `n_q` is final here
                cols.push((n_outer + n_q + w.len()) as u32);
                w.push(t);
                // the tape works in the units a person writes (degrees); (m, c) are the
                // conversion to what the kernel reads, the same seam `expr::set_dimension` is
                rows.push(Row {
                    kid: twin as usize,
                    cols,
                    consts: vec![crate::expr::to_arg_units(k, 1.0), 0.0],
                });
                continue;
            }
            None => c.kernel_id(),
        };
        let consts = c.consts_on(&sk, None);
        rows.push(Row { kid, cols, consts });
    }
    let traced = match scope.get(point) {
        Some(e) if e.kind == EntKind::Point => {
            let p = sk.point_params(e.i())[0];
            match slot.get(&p) {
                Some(&s) if s >= n_outer => s - n_outer,
                _ => {
                    return Err((
                        span,
                        format!("`{point}` must be a point the component declares"),
                    ))
                }
            }
        }
        _ => {
            return Err((span, format!("`{point}` must be a point the component declares")))
        }
    };
    let locus = Locus::new(n_outer, n_theta, n_q, traced, w, seeds, rows, preds)
        .map_err(|m| (span, m))?;
    Ok((locus, pose_of))
}

/// A seed named geometrically, compiled to the tapes a written pair would be: the place a point
/// already names, or **the point at the edge of a circle** at a bearing from the page's x-axis —
/// `hint(at: c, bearing: u + phase)`, which is what that place is called in this language rather
/// than the trigonometry it comes to.
fn at_seed(
    sk: &Sketch,
    scope: &BTreeMap<String, EntRef>,
    slot: &BTreeMap<u32, usize>,
    vars: &[String],
    seeds: &[crate::tape::Tape],
    n_outer: usize,
    a: &crate::syntax::AtRef,
    span: Span,
) -> Result<(crate::tape::Tape, crate::tape::Tape), (Span, String)> {
    use crate::expr::{Ast, Op};
    use crate::tape::Tape;
    let e = scope
        .get(&a.what.root.text)
        .copied()
        .ok_or((a.what.span, format!("no such entity: `{}`", a.what.root.text)))?;
    let e = follow(sk, e, &a.what.path).map_err(|m| (a.what.span, m))?;
    // what one scratch parameter is seeded from: a formal's coordinate is a variable of the
    // family; an inner point's is whatever seeded it, which is why declaration order matters
    let of = |p: u32| -> Result<Tape, (Span, String)> {
        let s = *slot.get(&p).ok_or((span, "trace lowering lost a column".to_string()))?;
        if s < n_outer {
            Tape::compile(&Ast::Var(vars[s].clone()), vars).map_err(|m| (span, m))
        } else {
            seeds.get(s - n_outer).cloned().ok_or((
                a.what.span,
                format!("`{}` is declared after this point", a.what.root.text),
            ))
        }
    };
    match (e.kind, &a.bearing) {
        (EntKind::Point, None) => {
            let ps = sk.point_params(e.i());
            Ok((of(ps[0])?, of(ps[1])?))
        }
        (EntKind::Point, Some(_)) => {
            Err((span, "a point is already a place; a bearing needs a circle".to_string()))
        }
        (EntKind::Circle, Some((text, bsp))) => {
            let c = &sk.circles[e.i()];
            // the edge of an *inner* circle would need its seed tapes composed with the
            // bearing's; the case nobody has asked for yet
            let name = |p: u32| -> Result<String, (Span, String)> {
                match slot.get(&p) {
                    Some(&s) if s < n_outer => Ok(vars[s].clone()),
                    _ => Err((
                        a.what.span,
                        "the circle must be one the family is written over".to_string(),
                    )),
                }
            };
            let ctr = sk.point_params(c.center as usize);
            let (cx, cy, r) = (name(ctr[0])?, name(ctr[1])?, name(c.radius)?);
            let beta = crate::expr::parse_in(text, sk.units).map_err(|m| (*bsp, m))?.body;
            let coord = |centre: &str, trig: &str| -> Result<Tape, (Span, String)> {
                let ast = Ast::Bin(
                    Op::Add,
                    Box::new(Ast::Var(centre.to_string())),
                    Box::new(Ast::Bin(
                        Op::Mul,
                        Box::new(Ast::Var(r.clone())),
                        Box::new(Ast::Call(trig.to_string(), vec![beta.clone()])),
                    )),
                );
                Tape::compile(&ast, vars).map_err(|m| (*bsp, m))
            };
            Ok((coord(&cx, "cos")?, coord(&cy, "sin")?))
        }
        (EntKind::Circle, None) => Err((
            span,
            "where on the edge?  `hint(at: c, bearing: …)` says the bearing".to_string(),
        )),
        (k, _) => Err((a.what.span, format!("a seed cannot be at a {}", k.as_str()))),
    }
}

/// Where an unseeded point starts — an implicit child, a declared point with no `hint(…)`.
///
/// **Nothing in the language says** (spec §15): a declaration with no hint has unknowns, and the
/// document says no more than that.  The implementation needs an answer all the same, and the
/// obvious one is wrong — `line l`'s two endpoints both at the origin is a zero-length line, with
/// no direction for `horizontal(l)` to bite on and a singular row for any tangency, and
/// `point a` / `point b` / `a distance(30) b` there is a stationary point of the one residual it
/// has.  So they scatter: distinct bearings round a unit circle, jittered off the crate's seeded
/// `rng::Rng` so the answer is the same on every run and on every machine.  No exception for
/// the drawing's first point: a point declared before `point base hint(x: 0, y: 0)` would sit
/// on it, and a seeded point at the origin is the commonest one there is.  It is an
/// implementation choice and belongs nowhere in the spec.
/// Where an unwritten radius starts when the geometry gives none — a circle's, or an arc whose
/// start sits on its centre.  Any length but zero would do (the row's gradient in `r` is `−2r`,
/// so the solve moves it from anywhere else); this one is the scale `scatter` places points at,
/// and as much an implementation choice as that is.
const UNSEEDED_RADIUS: f64 = 1.0;

fn scatter(i: usize) -> (f64, f64) {
    // the bearing walks a fixed step per minted point, in creation order — which for a chain's
    // corners is traversal order, so a contour of implicit points seeds as a *simple polygon*
    // and not as a pile or a self-crossing quad, whose nearest solution is a collapsed side (a
    // zero-length line satisfies every direction constraint on it).  The step is irrational in
    // turns (half the golden angle), so no later point lands back on an earlier bearing.
    const STEP: f64 = 1.199982;
    let mut rng = Rng::new(0x5eed_u32 ^ (i as u32).wrapping_mul(2_654_435_761));
    let th = i as f64 * STEP + rng.uniform(-0.2, 0.2);
    let r = rng.uniform(0.8, 1.2);
    (r * th.cos(), r * th.sin())
}

/// The dotted names of a kind's child slots: `l.p1`, `a.center`.  An anonymous child has no name
/// in the source, so this **is** its name, and everything that identifies an entity by name —
/// the map, a new constraint's arguments, a selection crossing a re-elaboration — asks here.
///
/// Under `base`, which is the declaration's own name — or, for an anonymous one, what the
/// *drawing* calls it (`shown`), since the key it resolves by is not a thing to show anybody.
pub(crate) fn child_names(d: &Decl, base: &str) -> Vec<String> {
    d.kind
        .fields()
        .iter()
        .filter(|(_, f)| *f == Field::Child)
        .map(|(n, _)| format!("{base}.{n}"))
        .collect()
}

/// What to *call* a declaration where a person will read it — the name it was given, and for an
/// anonymous one the positional name the drawing already labels it by (`l0`, `p2`).
///
/// The key an anonymous declaration resolves by is an offset, and a parameter carries its
/// entity's name into every place a parameter is listed (a DOF report, a mode's label), so the
/// key would be read by somebody.  Names in the *sketch* are display; names in the source map
/// are identity; this is the one seam where the two part company.
///
/// `DeclName::shown`, and not any reading of the characters: a **block prefix** is a name the
/// flattener made and has always been shown — it says which instance the thing belongs to, which
/// an index cannot — so only a declaration the source named *nothing* is relabelled here.  A
/// predicate over the string can tell those two apart only by the marker the anonymous mint
/// happens to use, and the broader of the two readings shows one entity by its path in one
/// window and by its index in another.
fn shown(sk: &Sketch, d: &Decl) -> String {
    match d.name.shown() {
        None => crate::syntax::entity_name(EntRef::new(d.kind, sk.count(d.kind))),
        Some(n) => n.text.clone(),
    }
}

/// Every plane's basis, resolved from what its declaration wrote (§6.7): the page, a fold
/// from another plane, or a basis given outright.  Memoised recursion over the `from` chain,
/// so a plane is worked out once and a cycle is caught where it closes.  A plane whose
/// attitude is refused has no entry, and `build` then leaves it unbuilt.
///
/// **Keyed by the declaration's name, never by its statement id.**  A statement expanded by
/// `flatten` keeps the id of the statement it came from, so a component instanced twice — or a
/// plane declared inside a `cycle` — is several planes from one id, each with the fold its own
/// copy was given (`settle_arg` substitutes the instance's parameters into the text).  Keyed by
/// id every copy read the first one's basis and came out silently wrong; the *name* is the
/// prefixed absolute one the resolver keys on, which is one per copy.
fn plane_bases(
    body: &[&Stmt],
    res: &Resolver,
    skip: &BTreeSet<StmtId>,
    units: crate::units::Units,
    diags: &mut Vec<Diag>,
) -> BTreeMap<String, crate::plane::Basis> {
    let mut decls: BTreeMap<&str, (&Stmt, &Decl)> = BTreeMap::new();
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        if d.kind == EntKind::Plane && !skip.contains(&st.id) {
            decls.entry(&d.name.key().text).or_insert((st, d));
        }
    }
    let mut done: BTreeMap<String, Option<crate::plane::Basis>> = BTreeMap::new();
    let keys: Vec<&str> = decls.keys().copied().collect();
    for k in keys {
        basis_of(k, &decls, res, units, &mut done, &mut Vec::new(), diags);
    }
    done.into_iter().filter_map(|(k, b)| b.map(|b| (k, b))).collect()
}

fn basis_of<'a>(
    key: &'a str,
    decls: &BTreeMap<&'a str, (&'a Stmt, &'a Decl)>,
    res: &Resolver,
    units: crate::units::Units,
    done: &mut BTreeMap<String, Option<crate::plane::Basis>>,
    stack: &mut Vec<&'a str>,
    diags: &mut Vec<Diag>,
) -> Option<crate::plane::Basis> {
    let &(st, d) = decls.get(key)?;
    if let Some(b) = done.get(key) {
        return *b;
    }
    let fail = |diags: &mut Vec<Diag>, code: Code, span: Span, message: String| {
        diags.push(Diag { code, span, stmt: Some(st.id), message });
    };
    // one number the attitude was written with, as the dimension its slot takes
    // what a written number comes to, asked of the one function that answers it everywhere —
    // an attitude is written in the same little language a dimension is, and read in the
    // document's own units.  Nothing is in scope: a plane's fold is settled per copy by the
    // flattener before this runs, so what arrives here is arithmetic over literals.
    let number = |a: &Arg, want: crate::units::Dim, what: &str| -> Result<f64, String> {
        let Arg::Dim { text, .. } = a else { return Err(format!("`{what}` is not a number")) };
        let v = crate::flatten::value_aff(text, &BTreeMap::new(), units)
            .map_err(|e| format!("`{text}`: {e}"))?;
        v.dim.require(want, what)?;
        Ok(v.c)
    };
    let basis = match &d.attitude {
        Attitude::Page => Some(crate::plane::Basis::page()),
        Attitude::From { plane, fold } => {
            let parent = if !plane.path.is_empty() {
                fail(diags, Code::E040, plane.span, "`from` names a plane, not a part of one".into());
                None
            } else {
                match res.lookup(plane) {
                    None => {
                        fail(
                            diags,
                            Code::E101,
                            plane.span,
                            format!("no such entity: `{}`", plane.root.text),
                        );
                        None
                    }
                    Some(e) if e.kind != EntKind::Plane => {
                        fail(
                            diags,
                            Code::E040,
                            plane.span,
                            format!(
                                "`{}` is a {}, and `from` names a plane",
                                plane.root.text,
                                e.kind.as_str()
                            ),
                        );
                        None
                    }
                    Some(_) if stack.contains(&key) || plane.root.text == key => {
                        fail(
                            diags,
                            Code::E041,
                            plane.span,
                            format!("`{key}` is folded from itself, through `{}`", plane.root.text),
                        );
                        None
                    }
                    Some(_) => {
                        stack.push(key);
                        let p = basis_of(plane.root.text.as_str(), decls, res, units, done,
                                         stack, diags);
                        stack.pop();
                        p
                    }
                }
            };
            let theta = match fold {
                None => Some(0.0),
                Some(a) => match number(a, crate::units::Dim::ANGLE, "fold") {
                    Ok(deg) => Some(expr::to_arg_units(SpecKind::Angle, deg)),
                    Err(m) => {
                        fail(diags, Code::E103, arg_span(a).unwrap_or(st.span), m);
                        None
                    }
                },
            };
            match (parent, theta) {
                (Some(p), Some(t)) => Some(p.fold(t)),
                _ => None,
            }
        }
        Attitude::Basis { u, v } => {
            let mut vals = [[0.0; 3]; 2];
            let mut ok = true;
            for (k, (name, triple)) in [("u", u), ("v", v)].into_iter().enumerate() {
                for (i, a) in triple.iter().enumerate() {
                    match number(a, crate::units::Dim::SCALAR, name) {
                        Ok(x) => vals[k][i] = x,
                        Err(m) => {
                            fail(diags, Code::E103, arg_span(a).unwrap_or(st.span), m);
                            ok = false;
                        }
                    }
                }
            }
            let b = ok.then(|| crate::plane::Basis::explicit(vals[0], vals[1])).flatten();
            if ok && b.is_none() {
                fail(
                    diags,
                    Code::E103,
                    arg_span(&u[0]).unwrap_or(st.span),
                    "`u` and `v` do not span a plane".into(),
                );
            }
            b
        }
    };
    done.insert(key.to_string(), basis);
    basis
}

/// `point a in top` (§6.7): every point a declaration mints or names is put on the plane it
/// names.  After every kind is built, since the plane may be declared after the point; before
/// any constraint, since `project` reads these.  A point two declarations put on two different
/// planes is E060 — one image is on one plane.
fn memberships(
    sk: &mut Sketch,
    res: &Resolver,
    map: &SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        let Some(r) = d.membership.plane() else { continue };
        if skip.contains(&st.id) {
            continue;
        }
        let mut fail = |code: Code, span: Span, message: String| {
            diags.push(Diag { code, span, stmt: Some(st.id), message });
        };
        let plane = match res.lookup(r) {
            None => {
                fail(Code::E101, r.span, format!("no such entity: `{}`", r.root.text));
                continue;
            }
            Some(e) if e.kind != EntKind::Plane || !r.path.is_empty() => {
                fail(
                    Code::E040,
                    r.span,
                    format!("`{}` is a {}, and `in` names a plane", r.root.text, e.kind.as_str()),
                );
                continue;
            }
            Some(e) => e.i(),
        };
        // a declaration that could not be built made nothing to put anywhere
        let Some(me) = res.of.get(&d.name.key().text).copied() else { continue };
        let points: Vec<usize> = match me.kind {
            EntKind::Point => vec![me.i()],
            _ => sk.children(me).into_iter().map(|k| k.i()).collect(),
        };
        for p in points {
            match sk.plane_of(p) {
                Some(q) if q != plane => {
                    let who = |e: EntRef| {
                        map.name_of(e).cloned().unwrap_or_else(|| io::entity_name(e))
                    };
                    fail(
                        Code::E060,
                        // the ref's own span: the clause's word, or the block header's
                        r.span,
                        format!(
                            "`{}` is already in `{}`",
                            who(EntRef::point(p)),
                            who(EntRef::plane(q))
                        ),
                    );
                }
                _ => sk.set_plane(p, Some(plane)),
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn build(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    bases: &BTreeMap<String, crate::plane::Basis>,
    diags: &mut Vec<Diag>,
    anon: &mut Vec<(String, EntRef)>,
    deferred: &mut Vec<Deferred>,
    prog: &Program,
    insts: &[crate::flatten::InstanceInfo],
) -> Option<EntRef> {
    // a curve is the one kind whose arguments need not be points, so it is built before the
    // walk that insists they are
    if d.kind == EntKind::Curve {
        return build_curve(sk, res, d, st, diags, prog, insts);
    }
    // a seed named by a place (`hint(at: k, bearing: b)`) is a point's; every other kind has
    // a scalar of its own the clause seeds by name
    if d.seed_at.is_some() && d.kind != EntKind::Point {
        diags.push(Diag {
            code: Code::E103,
            span: st.span,
            stmt: Some(st.id),
            message: "only a point takes a geometric seed (`hint(at: …)`)".to_string(),
        });
        return None;
    }
    // every child a declaration names, flattened in field order and checked to be a Point —
    // which every other child of every other kind is, and which is what an alias class must
    // agree about.  A slot may hold a *seed* instead of a name, and a declaration may write no
    // list at all: both mint a point nothing names, reached as `l.p1` (spec §6.1, §6.2).
    let written: usize = d.children.iter().map(|g| g.len()).sum();
    // the dotted names, worked out only where one is needed: every child a document names is a
    // `Kid::Ref`, and formatting names nothing would read is a string per slot per elaboration
    let anonymous = written == 0
        || d.children.iter().any(|g| g.is_empty() || g.iter().any(|k| matches!(k, Kid::Hint(_))));
    // The child's **name**, which is its dotted path, and what to **call** it — the same string
    // for a declaration the source named, and two different ones for an anonymous declaration,
    // whose key is an offset nobody should be shown.
    let dotted = if anonymous { child_names(d, &d.name.key().text) } else { Vec::new() };
    let label = if anonymous { child_names(d, &shown(sk, d)) } else { Vec::new() };
    let mut kids: Vec<usize> = Vec::new();
    // `Some(0)` and `None` both mean there is nothing to mint, and so does a written list
    let mint = if written == 0 { d.kind.children_arity().unwrap_or(0) } else { 0 };
    for k in 0..mint {
        let (x, y) = scatter(sk.points.len());
        // the dotted path *is* the point's name — there is no other — so it is what the map
        // binds; the sketch carries what a reader is shown
        let i = sk.point(x, y, false, &label[k]);
        kids.push(i);
        anon.push((dotted[k].clone(), EntRef::point(i)));
    }
    let mut slot = 0usize;
    for group in &d.children {
        // a slot nothing names or seeds is an *implicit child*, minted exactly as a declaration
        // that writes no list at all mints them (spec §6.2) — which is what lets a chain's
        // thread fill only the slots it speaks for (`line l1 -> line l2`) and leave the rest
        // the drawing's own
        if group.is_empty() && written != 0 {
            // a `List` kind has no arity to mint from, and a slot with no dotted path has no
            // name to be reached by
            if let Some(name) = d.kind.children_arity().and(dotted.get(slot)) {
                let (x, y) = scatter(sk.points.len());
                let i = sk.point(x, y, false, label.get(slot).unwrap_or(name));
                kids.push(i);
                anon.push((name.clone(), EntRef::point(i)));
            }
            slot += 1;
            continue;
        }
        for kid in group {
            let r = match kid {
                Kid::Ref(r) => r,
                Kid::Hint(seed) => {
                    // a list slot has no arity, so it has no dotted path to be named by, and a
                    // point nothing can name is a point nothing can constrain or drag
                    let Some(name) = dotted.get(slot) else {
                        diags.push(Diag {
                            code: Code::E103,
                            span: st.span,
                            stmt: Some(st.id),
                            message: format!(
                                "a {}'s control points have no names to be reached by, so each \
                                 one is declared",
                                d.kind.as_str()
                            ),
                        });
                        return None;
                    };
                    let i = sk.point(seed.v[0], seed.v[1], false, label.get(slot).unwrap_or(name));
                    for (k, t) in seed.text.iter().enumerate() {
                        if let Some(t) = t {
                            deferred.push(Deferred::Text {
                                param: sk.point_params(i)[k],
                                text: t.clone(),
                                names: d.seed_names.clone(),
                                span: seed.spans[k],
                                stmt: st.id,
                            });
                        }
                    }
                    kids.push(i);
                    anon.push((name.clone(), EntRef::point(i)));
                    slot += 1;
                    continue;
                }
            };
            slot += 1;
            let Some(e) = res.lookup(r) else {
                diags.push(Diag {
                    code: Code::E101,
                    span: r.span,
                    stmt: Some(st.id),
                    message: format!("no such entity: `{}`", r.root.text),
                });
                return None;
            };
            let e = match follow_building(sk, res, e, r) {
                Ok(e) => e,
                Err(msg) => {
                    diags.push(Diag {
                        code: Code::E040,
                        span: r.span,
                        stmt: Some(st.id),
                        message: msg,
                    });
                    return None;
                }
            };
            if e.kind != EntKind::Point {
                diags.push(Diag {
                    code: Code::E040,
                    span: r.span,
                    stmt: Some(st.id),
                    message: format!(
                        "`{}` is a {}, and a {} is built from points",
                        r.root.text,
                        e.kind.as_str(),
                        d.kind.as_str()
                    ),
                });
                return None;
            }
            kids.push(e.i());
        }
    }
    // a slot carries a name, a seed, or nothing — an implicit child, minted above — so the one
    // thing left to refuse is *more* than the kind has slots for
    let want = d.kind.children_arity();
    if let Some(n) = want {
        if kids.len() != n {
            diags.push(Diag {
                code: Code::E103,
                span: st.span,
                stmt: Some(st.id),
                message: format!(
                    "a {} is built from {n} point(s), and {} were given",
                    d.kind.as_str(),
                    written
                ),
            });
            return None;
        }
    }
    let seed = |i: usize| d.seed.get(i).copied().unwrap_or(0.0);
    // what a reader is shown: the declaration's own name, or what the drawing calls it where
    // the source named nothing — a scalar carries this into every list of parameters
    let show = shown(sk, d);
    // A point whose source wrote no seed at all — no `hint(…)` clause (the empty span where
    // one would go) and no place — starts where a minted child does, not at the origin: two
    // such points on top of each other put every distance between them at a stationary point
    // of its own residual, and the first document anybody writes solved as a conflict (#43).
    // A declaration lifted from a sketch has no span (`None`) and carries its numbers.
    let unseeded = d.seed_at.is_none() && d.hint_span.is_some_and(|s| s.is_empty());
    // A scalar the source never wrote reads as 0, and for a radius 0 is a stationary point of
    // every on-circle row (∂/∂r of |p−c|² − r² is −2r): an `arc` with its ends grounded and no
    // `hint(r:)` could not solve at all, and a conflict elsewhere in the drawing was blamed on
    // the arc's own intrinsic, the first row a search from that pose could not satisfy (#45.6).
    // So a radius is *written or computed*, never defaulted: the constructor's geometric one for
    // an arc, `UNSEEDED_RADIUS` for a circle and wherever the
    // geometry gives nothing.  A declaration lifted from a sketch has no spans and carries its
    // numbers, as for a point above.
    let wrote = |i: usize| {
        d.hint_span.is_none() || d.seed_spans.get(i).is_some_and(|s| !s.is_empty())
    };
    let nonzero = |r: f64| if r.abs() > 1e-9 { r } else { UNSEEDED_RADIUS };
    let idx = match d.kind {
        EntKind::Point if unseeded => {
            let (x, y) = scatter(sk.points.len());
            sk.point(x, y, false, &show)
        }
        EntKind::Point => sk.point(seed(0), seed(1), false, &show),
        EntKind::Line => sk.line(kids[0], kids[1]),
        EntKind::Circle => {
            let r = if wrote(0) { seed(0) } else { UNSEEDED_RADIUS };
            sk.circle(kids[0], r, &show)
        }
        EntKind::Arc => {
            // `arc` adds the two intrinsic `PointOnCircle`s here and nowhere else, and computes a
            // radius from the geometry that a *written* seed then replaces
            let ai = sk.arc(kids[0], kids[1], kids[2], &show);
            let rp = sk.arcs[ai].radius as usize;
            sk.params[rp].value =
                if wrote(0) { seed(0) } else { nonzero(sk.params[rp].value) };
            ai
        }
        EntKind::Spline => {
            if kids.len() > io::MAX_CTRL {
                diags.push(Diag {
                    code: Code::E104,
                    span: st.span,
                    stmt: Some(st.id),
                    message: format!("a curve may not have more than {} control points", io::MAX_CTRL),
                });
                return None;
            }
            match sk.spline_with(&kids, d.knots.clone()) {
                Some(si) => si,
                None => {
                    diags.push(Diag {
                        code: Code::E103,
                        span: st.span,
                        stmt: Some(st.id),
                        message: format!(
                            "a curve needs more than {} control points and a matching knot vector",
                            curve::DEGREE
                        ),
                    });
                    return None;
                }
            }
        }
        EntKind::Plane => {
            // `plane` adds the two intrinsics here and nowhere else, and computes a rotor from
            // the chord that a declared seed then replaces — except (0, 0), which is no rotor
            // at all and is what an unwritten seed reads as — over a basis the attitude pass
            // resolved before this walk; a plane whose basis was refused has no entry and is
            // not built
            let basis = *bases.get(&d.name.key().text)?;
            let pi = sk.plane(kids[0], kids[1], basis, &show);
            let (c, s) = (seed(0), seed(1));
            if c != 0.0 || s != 0.0 {
                let f = &sk.planes[pi].frame;
                let (cp, sp) = (f.c as usize, f.s as usize);
                sk.params[cp].value = c;
                sk.params[sp].value = s;
            }
            pi
        }
        EntKind::Curve => unreachable!("a curve is built before this walk"),
    };
    let e = EntRef::new(d.kind, idx);
    set_class(sk, e, d.class.clone());
    // the seeds this declaration wrote over geometry, settled once everything is built: a
    // point's place, and any scalar's text that read another entity's
    if let Some(at) = &d.seed_at {
        deferred.push(Deferred::At {
            point: idx,
            at: at.clone(),
            names: d.seed_names.clone(),
            span: st.span,
            stmt: st.id,
        });
    }
    // the declaration's own scalars are the last of `entity_params`, after its children's
    let params = sk.entity_params(e);
    let own = &params[params.len().saturating_sub(d.seed_text.len())..];
    for (k, t) in d.seed_text.iter().enumerate() {
        if let (Some(t), Some(param)) = (t, own.get(k).copied()) {
            let span = d.seed_spans.get(k).copied().unwrap_or(st.span);
            deferred.push(Deferred::Text {
                param,
                text: t.clone(),
                names: d.seed_names.clone(),
                span,
                stmt: st.id,
            });
        }
    }
    Some(e)
}

/// A seed that reads geometry, held until every declaration has a seed to be read (§6.4).
///
/// Two spellings, one rule: **the value read is the other scalar's own seed**.  A text over
/// entity scalars — `hint(x: k.center.x + k.r, y: pin.y)` — and a place named outright —
/// `hint(at: pin)`, `hint(at: k, bearing: b)`, the words a trace block already had for the same
/// two things.  Settled in statement order, so a seed that reads a seed that was itself read from a
/// third comes out right when the three are written in the order they depend on; written the
/// other way round it reads the earlier one's *provisional* seed, which is the same bargain the
/// trace block's "declared after this point" strikes, said more leniently.
enum Deferred {
    Text { param: u32, text: String, names: Vec<(String, String)>, span: Span, stmt: StmtId },
    At { point: usize, at: AtRef, names: Vec<(String, String)>, span: Span, stmt: StmtId },
}

/// The seed a dotted name reads: `pin.x`, `k.center.y`, `base.r`, `e.b` — `dotted` as the
/// flattener resolved it, its root the entity's absolute name, and the last segment the scalar by
/// the kind's own `scalar_names`.
fn seed_read(sk: &Sketch, res: &Resolver, dotted: &str) -> Result<f64, String> {
    let (path, scalar) = dotted
        .rsplit_once('.')
        .ok_or_else(|| format!("`{dotted}` is not a number here"))?;
    // an entity's absolute name is dotted itself (`t1.pin` under an instance), so the entity is
    // the longest head of the path that names one, and the rest is fields into it
    let segs: Vec<&str> = path.split('.').collect();
    let (e, fields) = (1..=segs.len())
        .rev()
        .find_map(|k| res.of.get(&segs[..k].join(".")).map(|e| (*e, &segs[k..])))
        .ok_or_else(|| format!("no such entity: `{}`", segs[0]))?;
    let fields: Vec<Seg> = fields.iter().map(|f| Seg::Field(Name::new(*f))).collect();
    let e = follow(sk, e, &fields)?;
    let names = e
        .kind
        .scalar_names(path)
        .ok_or_else(|| format!("a {} has no scalar to read by name", e.kind.as_str()))?;
    let at = names.iter().position(|n| *n == dotted).ok_or_else(|| {
        format!(
            "a {} has no `{scalar}`; its scalars are {}",
            e.kind.as_str(),
            names.iter().map(|n| format!("`{n}`")).collect::<Vec<_>>().join(", ")
        )
    })?;
    let p = *sk
        .entity_params(e)
        .get(at)
        .ok_or_else(|| format!("`{dotted}` has no seed yet"))?;
    Ok(sk.params[p as usize].value)
}

/// An expression over geometry's seeds, come to its number.  `names` is what each dotted name
/// in it resolved to (`Decl::seed_names`); one it does not list is read as written.
fn seed_eval(
    sk: &Sketch,
    res: &Resolver,
    text: &str,
    names: &[(String, String)],
) -> Result<f64, String> {
    let p = expr::parse_in(text, sk.units)?;
    let mut env: BTreeMap<String, expr::Aff> = BTreeMap::new();
    // a coordinate or a radius is a length, so it adds to `150mm` and not to a plain number —
    // where the document names a unit.  Where it does not, no literal can be a length, so the
    // read is the bare number everything else there is (§3.3).
    let dim = if sk.units.name().is_some() {
        crate::units::Dim::LENGTH
    } else {
        crate::units::Dim::SCALAR
    };
    for dep in p.body.deps() {
        if dep.contains('.') {
            let abs = names.iter().find(|(w, _)| *w == dep).map(|(_, a)| a.as_str());
            let v = seed_read(sk, res, abs.unwrap_or(&dep))?;
            env.insert(dep.clone(), expr::Aff::of_dim(v, dim));
        }
    }
    let a = expr::eval(&p.body, &env)?;
    match a.number() {
        Some(v) if v.is_finite() => Ok(v),
        Some(v) => Err(format!("comes to {v}")),
        None => Err(format!(
            "`{}` is not a number here — a component's parameters are, and the document's \
             dimensions are not",
            a.free.unwrap_or_default()
        )),
    }
}

fn settle_deferred(sk: &mut Sketch, res: &Resolver, deferred: &[Deferred], diags: &mut Vec<Diag>) {
    for d in deferred {
        let (span, stmt, result) = match d {
            Deferred::Text { param, text, names, span, stmt } => {
                let r = seed_eval(sk, res, text, names).map(|v| {
                    sk.params[*param as usize].value = v;
                });
                (*span, *stmt, r.map_err(|e| format!("`{text}`: {e}")))
            }
            Deferred::At { point, at, names, span, stmt } => {
                let r = place_of(sk, res, at, names).map(|(x, y)| {
                    let [px, py] = sk.point_params(*point);
                    sk.params[px as usize].value = x;
                    sk.params[py as usize].value = y;
                });
                (*span, *stmt, r)
            }
        };
        if let Err(message) = result {
            diags.push(Diag { code: Code::E103, span, stmt: Some(stmt), message });
        }
    }
}

/// Where `hint(at: …)` puts a point on the sheet: the seed of the point it names, or the edge
/// of the circle it names at the bearing given — `at_seed`'s two places, read as numbers.
fn place_of(
    sk: &Sketch,
    res: &Resolver,
    a: &AtRef,
    names: &[(String, String)],
) -> Result<(f64, f64), String> {
    let e = *res
        .of
        .get(&a.what.root.text)
        .ok_or_else(|| format!("no such entity: `{}`", a.what.root.text))?;
    let e = follow(sk, e, &a.what.path)?;
    match (e.kind, &a.bearing) {
        (EntKind::Point, None) => Ok(sk.point_xy(e.i())),
        (EntKind::Point, Some(_)) => {
            Err("a point is already a place; a bearing needs a circle".to_string())
        }
        (EntKind::Circle, Some((text, _))) => {
            let c = &sk.circles[e.i()];
            let (cx, cy) = sk.point_xy(c.center as usize);
            let r = sk.params[c.radius as usize].value;
            // a bearing is an angle, so the text may say `90deg` or read a `param` already
            // written in; what it comes to is in the document's angle unit, which is degrees
            let b = seed_eval(sk, res, text, names).map_err(|m| format!("`{text}`: {m}"))?;
            let b = b.to_radians();
            Ok((cx + r * b.cos(), cy + r * b.sin()))
        }
        (EntKind::Circle, None) => {
            Err("where on the edge?  `hint(at: c, bearing: …)` says the bearing".to_string())
        }
        (k, _) => Err(format!("a seed cannot be at a {}", k.as_str())),
    }
}


/// A curve instance: a family, the entities it is written over, and the numbers it takes.
fn build_curve(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    diags: &mut Vec<Diag>,
    prog: &Program,
    insts: &[crate::flatten::InstanceInfo],
) -> Option<EntRef> {
    match curve_entity(sk, res, d, st, prog, insts) {
        Ok(Some(cv)) => {
            sk.curves.push(cv);
            Some(EntRef::new(EntKind::Curve, sk.curves.len() - 1))
        }
        Ok(None) => None,
        Err((code, span, message)) => {
            diags.push(Diag { code, span, stmt: Some(st.id), message });
            None
        }
    }
}

/// The curve a declaration draws: its definition (compiled on first use, shared after), the
/// entities and numbers the instance gave it, and where its trace is anchored.  `Ok(None)` when
/// the flattener already reported what is wrong with it.
fn curve_entity(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    prog: &Program,
    insts: &[crate::flatten::InstanceInfo],
) -> Result<Option<crate::model::CurveE>, (Code, Span, String)> {
    let Some(c) = d.curve.as_ref() else {
        return Err((
            Code::E103,
            st.span,
            "a curve is `curve name = instance.point over formal in (a, b)`".to_string(),
        ));
    };
    // the instance the point belongs to — found by the flattener, which reported it if it
    // found none
    let Some(of) = &c.of else { return Ok(None) };
    let Some(info) = insts.iter().find(|i| i.prefix == of.instance) else { return Ok(None) };
    let Some(comp) = prog.component(&info.component) else { return Ok(None) };
    let swept = c.swept.text.as_str();
    let numeric = comp
        .formals
        .iter()
        .any(|f| f.name.text == swept && !matches!(f.ty, crate::syntax::Ty::Ent(_)));
    if !numeric {
        return Err((
            Code::E040,
            c.swept.span,
            format!("`{swept}` is not a numeric formal of `{}`", info.component),
        ));
    }
    // one definition per (component, point, formal), shared by every instance asked for it
    let key = crate::model::CurveDef::key(&info.component, &of.point, swept);
    let di = match sk.curve_defs.iter().position(|x| x.name == key) {
        Some(i) => i,
        None => {
            let def = compile_curve(prog, comp, swept, &of.point, sk.units)
                .map_err(|(span, m)| (Code::E103, span, m))?;
            sk.curve_defs.push(def);
            sk.curve_defs.len() - 1
        }
    };
    let def = &sk.curve_defs[di];
    let missing = |what: &str| {
        (Code::E101, st.span, format!("`{}` was not given `{what}`", info.component))
    };
    // the entities the instance was given, in the component's order
    let mut args: Vec<EntRef> = Vec::with_capacity(def.formals.len());
    for ((fname, k), (_, actual)) in def.formals.iter().zip(&info.ents) {
        let abs = actual.as_ref().ok_or_else(|| missing(fname))?;
        let e = *res
            .of
            .get(abs)
            .ok_or_else(|| (Code::E101, st.span, format!("no such entity: `{abs}`")))?;
        if e.kind != *k {
            return Err((
                Code::E040,
                st.span,
                format!("`{fname}` is a {}, and a {} was given", k.as_str(), e.kind.as_str()),
            ));
        }
        args.push(e);
    }
    // the numbers it was given, in the definition's order
    let values = def
        .values
        .iter()
        .map(|n| info.values.get(n).and_then(|a| a.number()).ok_or_else(|| missing(n)))
        .collect::<Result<Vec<f64>, _>>()?;
    let num = |t: &str| crate::expr::literal(t);
    let domain = (num(&c.domain.0).unwrap_or(0.0), num(&c.domain.1).unwrap_or(1.0));
    // the anchor: what the swept formal was given — a number, or the drawing's unknown a drawn
    // instance left it as — or the interval's start for an instance written in place
    let home = match info.values.get(swept).map(|a| (a.number(), &a.free)) {
        Some((Some(v), _)) => crate::model::Home::At(v),
        Some((None, Some(n))) if info.drawn => crate::model::Home::Free(n.clone()),
        _ => crate::model::Home::At(domain.0),
    };
    // the pose a drawn instance stands in, per inner unknown of the block — whole or nothing
    let pose: Vec<(EntRef, usize)> = if info.drawn {
        def.pose_of
            .iter()
            .filter_map(|(n, j)| res.of.get(&format!("{}{n}", of.instance)).map(|&e| (e, *j)))
            .collect()
    } else {
        Vec::new()
    };
    let pose = crate::model::whole(pose, def.pose_of.len());
    Ok(Some(crate::model::CurveE {
        def: di as u32,
        args,
        values,
        domain,
        home,
        pose,
        class: d.class.clone(),
    }))
}

fn set_class(sk: &mut Sketch, e: EntRef, c: Classes) {
    match e.kind {
        EntKind::Point => {}
        EntKind::Line => sk.lines[e.i()].class = c,
        EntKind::Curve => sk.curves[e.i()].class = c,
        EntKind::Circle => sk.circles[e.i()].class = c,
        EntKind::Arc => sk.arcs[e.i()].class = c,
        EntKind::Spline => sk.splines[e.i()].class = c,
        EntKind::Plane => sk.planes[e.i()].frame.class = c,
    }
}

/// The constraint an operator statement names, and its arguments in spec order.
///
/// Two lookups and an assembly: `constraints::infix_op` / `prefix_op` turn the word and the
/// operands' kinds into a `CKind`, and `Written::assemble` puts what was written into the slots
/// that kind has.  Both tables are exhaustive over the library, so a new constraint joins the
/// language by being given an operator and nothing else.
fn settle(
    w: &crate::syntax::Written,
    kind_of: &dyn Fn(&Ref) -> Option<EntKind>,
) -> Result<(CKind, Vec<Option<Arg>>), (Span, String)> {
    use crate::constraints::Fixity;
    let word = w.word.text.as_str();
    // a gauge or an orientation is settled by its word alone: `fix c.r` names a number and not
    // an entity, and `ccw(a, b, c)` has no operand outside its parentheses; what each operand
    // must be is checked where the statement is applied, in the words the gauges always used
    if let Some(k) = crate::constraints::gauge_op(word) {
        return Ok((k, w.assemble(k)?));
    }
    let kinds: Vec<Option<EntKind>> = w.ops.iter().map(kind_of).collect();
    // a name that resolves to nothing is reported on the argument itself, where the message can
    // say which name it was; here it only means the word cannot be settled
    let named = |k: Option<EntKind>| k.map(|k| k.as_str()).unwrap_or("that");
    let kind = match w.fixity {
        Fixity::Prefix => {
            let Some(a) = kinds.first().copied().flatten() else {
                let m = format!("`{word}` needs to know what `{}` is", w.ops[0].root.text);
                return Err((w.word.span, m));
            };
            crate::constraints::prefix_op(word, a).ok_or_else(|| {
                (w.word.span, format!("`{word}` does not apply to a {}", a.as_str()))
            })?
        }
        Fixity::Infix => {
            let (a, b) = (kinds.first().copied().flatten(), kinds.get(1).copied().flatten());
            let (Some(a), Some(b)) = (a, b) else {
                return Err((w.word.span, format!("`{word}` needs to know what its operands are")));
            };
            crate::constraints::infix_op(word, a, b, &|n| w.sel(n)).ok_or_else(|| {
                let m =
                    format!("`{word}` does not relate a {} to a {}", named(Some(a)), named(Some(b)));
                (w.word.span, m)
            })?
        }
        // only a gauge word is written as a call, and those were settled above
        Fixity::Call => return Err((w.word.span, format!("`{word}` is not a call"))),
    };
    Ok((kind, w.assemble(kind)?))
}

fn constrain(
    sk: &mut Sketch,
    res: &Resolver,
    r: &Relation,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<u32> {
    // **The operator, settled.**  What a word means is the *kinds* of its operands — `on` is
    // five constraints, `distance` is six — and a name's kind is not known until here, so every
    // statement a document contains arrives as a word and its parentheses.  Settled before the
    // spec is read, since the spec is what the arguments are then checked against.
    let settled = match &r.poly {
        None => None,
        Some(w) => match settle(w, &|r| {
            res.lookup(r).and_then(|e| follow(sk, e, &r.path).ok()).map(|e| e.kind)
        }) {
            Ok(v) => Some(v),
            Err((span, message)) => {
                diags.push(Diag { code: Code::E040, span, stmt: Some(st.id), message });
                return None;
            }
        },
    };
    let (ckind, built);
    let args_of: &[Option<Arg>] = match &settled {
        Some((k, a)) => {
            ckind = *k;
            built = a.clone();
            &built
        }
        None => {
            ckind = r.kind;
            &r.args
        }
    };
    let r = &Relation { kind: ckind, args: args_of.to_vec(), ..r.clone() };
    // a gauge is applied, not added: it holds parameters or records a root choice, and there
    // is no constraint for the map to know it by.  A claim on one is refused below the way any
    // unclaimable kind is, so it is checked first.
    if ckind.gauge() {
        if r.claim {
            diags.push(Diag {
                code: Code::E040,
                span: st.span,
                stmt: Some(st.id),
                message: format!(
                    "`{}` is judged by nothing a claim can weigh: it holds a number or picks a \
                     root, and adds no row for the diagnosis to rank",
                    crate::syntax::snake(ckind.name())
                ),
            });
            return None;
        }
        apply_gauge(sk, res, r, st, diags);
        return None;
    }
    let spec = ckind.spec();
    let mut args: Vec<CArg> = Vec::with_capacity(spec.len());
    let mut left_out = vec![false; spec.len()];
    for (i, (name, kind)) in spec.iter().enumerate() {
        let given = r.args.get(i).and_then(|a| a.as_ref());
        let Some(a) = given else {
            left_out[i] = true;
            args.push(ckind.default_arg(i));
            continue;
        };
        match to_arg(sk, res, *kind, a) {
            Ok(v) => args.push(v),
            Err(msg) => {
                diags.push(Diag {
                    code: if msg.starts_with("no such") { Code::E101 } else { Code::E040 },
                    span: arg_span(a).unwrap_or(st.span),
                    stmt: Some(st.id),
                    message: format!("{}: {msg}", name),
                });
                return None;
            }
        }
    }
    // a magnitude stated negative: the kernel would square the sign away and the drawing show
    // the positive, so the document and the drawing would disagree about what the thing is
    if ckind.magnitude() {
        if let Some(i) = spec.iter().position(|(_, k)| *k == SpecKind::Length) {
            if args[i].num() < 0.0 {
                diags.push(Diag {
                    code: Code::E040,
                    span: r.args.get(i).and_then(|a| a.as_ref()).and_then(arg_span).unwrap_or(st.span),
                    stmt: Some(st.id),
                    message: format!(
                        "a {} is a magnitude and cannot be negative",
                        crate::syntax::snake(ckind.name())
                    ),
                });
                return None;
            }
        }
    }
    // `distance` between two circles is the radial gap between *concentric* ones — a kernel
    // that reads two radii and neither centre (`AnnularDistance`).  Written over two circles
    // centred apart, it says nothing about the gap a person meant and then duplicates the two
    // radii it does read (#43.21), so it is refused with the reading it has.
    if ckind == CKind::AnnularDistance {
        let centre = |e: EntRef| sk.children(e).first().copied();
        if centre(args[0].ent()) != centre(args[1].ent()) {
            diags.push(Diag {
                code: Code::E040,
                span: st.span,
                stmt: Some(st.id),
                message: "`distance` between two circles is the radial gap between concentric \
                          ones, and these are centred on different points — dimension the \
                          centres, or make the circles concentric"
                    .to_string(),
            });
            return None;
        }
    }
    // a claim is judged, never solved for, so it may own no unknown — `CKind::claimable` is the
    // rule, shared with the document readers; elaboration's job is only to give it a span
    if r.claim && !ckind.claimable() {
        diags.push(Diag {
            code: Code::E040,
            span: st.span,
            stmt: Some(st.id),
            message: format!(
                "`{}` carries an unknown of its own, and a claim may add none",
                crate::syntax::snake(ckind.name())
            ),
        });
        return None;
    }
    // the inferred slots the source left out — read off the geometry, the one place that rule
    // lives, shared with the document reader and the bindings' constraint records — and what
    // the model refuses once they are in, in its own words, given this statement's span
    if let Err(message) = io::seed_omitted(sk, ckind, &mut args, |i| left_out[i]) {
        diags.push(Diag { code: Code::E061, span: st.span, stmt: Some(st.id), message });
        return None;
    }
    let mut c = Constraint::new(ckind, args);
    c.claim = r.claim;
    c.class = r.class.clone();
    Some(sk.add_quiet(c))
}

fn arg_span(a: &Arg) -> Option<Span> {
    match a {
        Arg::Ref(r) => Some(r.span),
        Arg::Dim { span, .. } => Some(*span),
        _ => None,
    }
}

/// The written forms of a plain value argument — an int, a flag, a word, a float.  One table,
/// read by `to_arg` and by `compile_trace`, so an integer in a `Float` slot means the same thing
/// in a component body and in a trace block.
fn scalar_arg(kind: SpecKind, a: &Arg) -> Option<CArg> {
    Some(match (kind, a) {
        (SpecKind::Int, Arg::Int(v)) => CArg::Int(*v),
        (SpecKind::Int, Arg::Num(v)) => CArg::Int(*v as i64),
        (SpecKind::Bool, Arg::Bool(b)) => CArg::Bool(*b),
        (SpecKind::Str, Arg::Word(w)) => CArg::Str(w.clone()),
        (SpecKind::Float, Arg::Num(v)) => CArg::Num(*v),
        (SpecKind::Float, Arg::Int(v)) => CArg::Num(*v as f64),
        _ => return None,
    })
}

/// An entity argument, resolved: follow the reference's path and check the kind — one statement
/// of the rule, shared by `to_arg` and `compile_trace`, so the two readers of a spec cannot
/// drift on what an entity slot accepts or how it says no.
fn ent_arg(
    sk: &Sketch,
    found: Option<EntRef>,
    kind: SpecKind,
    r: &Ref,
) -> Result<CArg, String> {
    let e = found.ok_or_else(|| format!("no such entity: `{}`", r.root.text))?;
    let e = follow(sk, e, &r.path)?;
    if !crate::constraints::kind_matches(kind, e.kind) {
        return Err(format!(
            "`{}` is a {}, and a {} is wanted here",
            r.root.text,
            e.kind.as_str(),
            kind.as_str()
        ));
    }
    Ok(CArg::Ent(e))
}

fn to_arg(sk: &Sketch, res: &Resolver, kind: SpecKind, a: &Arg) -> Result<CArg, String> {
    if let Some(v) = scalar_arg(kind, a) {
        return Ok(v);
    }
    Ok(match (kind, a) {
        (k, Arg::Ref(r)) if k.is_entity() => ent_arg(sk, res.lookup(r), k, r)?,
        // a dimension: the text as written, handed to `expr.rs`, which owns that little language
        (k, Arg::Dim { text, .. }) if k.is_dimension() => {
            if text.len() > expr::MAX_TEXT {
                return Err(format!("longer than {} characters", expr::MAX_TEXT));
            }
            match expr::literal(text) {
                Some(n) => CArg::Num(expr::to_arg_units(k, n)),
                None => {
                    // in the *document's* units: `80mm` is a number here only where the document
                    // said what a number is, and saying so is `unit mm` (spec §3.3)
                    expr::parse_in(text, sk.units).map_err(|e| e.to_string())?;
                    CArg::Expr(expr::Expr::new(text.trim().to_string(), 0.0))
                }
            }
        }
        (k, Arg::Num(v)) if k.is_dimension() => CArg::Num(expr::to_arg_units(k, *v)),
        (SpecKind::Param, Arg::Seed { value, pinned }) => {
            CArg::Seed { value: *value, pinned: *pinned }
        }
        // expansion turns one of these into a `Seed`; one that reaches here was written outside
        // any component, where there are no parameters for it to be over
        (SpecKind::Param, Arg::SeedExpr { text, pinned, .. }) => CArg::Seed {
            value: expr::literal(text)
                .ok_or_else(|| format!("`{text}` is not a number this contact can start at"))?,
            pinned: *pinned,
        },
        (k, other) => return Err(format!("a {} is wanted here, not {other:?}", k.as_str())),
    })
}

/// A gauge or an orientation predicate, **applied** (issue #47, item 5): written and settled as
/// every other relation — an operator, a class, a placement — but holding parameters or
/// recording a root choice instead of becoming a constraint the sketch holds, so `constrain`
/// returns no id for it and the map never knows it.  The checks are the ones the two statement
/// kinds always made, in their words.
fn apply_gauge(sk: &mut Sketch, res: &Resolver, r: &Relation, st: &Stmt, diags: &mut Vec<Diag>) {
    let refs: Vec<&Ref> = r
        .args
        .iter()
        .filter_map(|a| match a {
            Some(Arg::Ref(rf)) => Some(rf),
            _ => None,
        })
        .collect();
    let mut bad = |code: Code, span: Span, message: String| {
        diags.push(Diag { code, span, stmt: Some(st.id), message })
    };
    match r.kind {
        CKind::Ground | CKind::Fix => {
            let Some(rf) = refs.first().copied() else {
                bad(Code::E103, st.span, format!("`{}` names what it pins", crate::syntax::snake(r.kind.name())));
                return;
            };
            let Some(e) = res.lookup(rf) else {
                bad(Code::E101, rf.span, format!("no such entity: `{}`", rf.root.text));
                return;
            };
            if r.kind == CKind::Ground {
                let e = follow(sk, e, &rf.path).unwrap_or(e);
                if e.kind != EntKind::Point {
                    bad(Code::E105, st.span, "ground pins a point; a scalar is pinned with fix".to_string());
                    return;
                }
                sk.fix_point(e.i(), true);
                return;
            }
            // `fix c.r`: the entity's own scalar, named by the field it is.  The document
            // stores one flag per scalar and nothing finer, so neither does this.
            let field = match rf.path.first() {
                Some(Seg::Field(f)) => f.text.clone(),
                _ => String::new(),
            };
            let own = sk.own_params(e);
            let scalars: Vec<&str> = e
                .kind
                .fields()
                .iter()
                .filter(|(_, f)| *f == Field::Scalar)
                .map(|(n, _)| *n)
                .collect();
            match scalars.iter().position(|&n| n == field) {
                Some(i) if i < own.len() => sk.params[own[i] as usize].fixed = true,
                _ => bad(
                    Code::E105,
                    st.span,
                    if scalars.is_empty() {
                        format!("a {} has no number of its own to fix", e.kind.as_str())
                    } else {
                        format!("a {} has {}, not `{field}`", e.kind.as_str(), scalars.join(" and "))
                    },
                ),
            }
        }
        CKind::Ccw | CKind::Cw => {
            if refs.len() != 3 {
                bad(Code::E103, st.span, "an orientation names three points".to_string());
                return;
            }
            let mut pts = [0usize; 3];
            for (i, rf) in refs.iter().enumerate() {
                match res.lookup(rf).and_then(|e| follow(sk, e, &rf.path).ok()) {
                    Some(e) if e.kind == EntKind::Point => pts[i] = e.i(),
                    _ => {
                        bad(Code::E101, rf.span, format!("no such point: `{}`", rf.root.text));
                        return;
                    }
                }
            }
            sk.branches.insert(decompose::branch_key(pts), if r.kind == CKind::Ccw { 1 } else { -1 });
        }
        _ => unreachable!("{:?} is not a gauge", r.kind),
    }
}

/* -- the lift ---------------------------------------------------------------------- */

/// The canonical program for a sketch.
///
/// Every `.json` document ever saved becomes a program through this, which is the whole of the
/// migration — and, while the parser is still being written, the whole of the bootstrap: a panel
/// can show a program before anything can read one back.
pub fn to_program(sk: &Sketch) -> Program {
    let mut p = Program::new();
    // what its numbers are in, first: every number after it is read in them (spec §3.3.2)
    if let Some(n) = sk.units.name() {
        p.push(StmtKind::Unit(Name::new(n)));
    }
    // the style sheet, before the geometry it styles
    for (name, style) in &sk.sheet {
        p.push(StmtKind::Style(crate::syntax::StyleRule {
            name: Name::new(name.clone()),
            style: style.clone(),
            // what this style states, asked of the style — a lifted rule has no source whose
            // wording it could be keeping
            props: style.stated().into_iter().map(|s| s.to_string()).collect(),
            span: Span::default(),
        }));
    }
    for e in sk.primitives() {
        p.push(StmtKind::Decl(lift_decl(sk, e)));
    }
    for c in sk.user_constraints() {
        p.push(StmtKind::Relation(lift_relation(sk, c)));
    }
    for i in 0..sk.points.len() {
        if sk.point_fixed(i) {
            p.push(StmtKind::Relation(lift_gauge(&entity_name(EntRef::point(i)), None)));
        }
    }
    for e in sk.primitives() {
        if e.kind == EntKind::Point {
            continue;
        }
        let own = sk.own_params(e);
        let scalars: Vec<&str> = e
            .kind
            .fields()
            .iter()
            .filter(|(_, f)| *f == Field::Scalar)
            .map(|(n, _)| *n)
            .collect();
        for (i, &pi) in own.iter().enumerate() {
            if sk.params[pi as usize].fixed {
                let f = scalars.get(i).copied().unwrap_or("r");
                p.push(StmtKind::Relation(lift_gauge(&entity_name(e), Some(f))));
            }
        }
    }
    for (key, &v) in &sk.branches {
        p.push(match decompose::branch_key_points(key) {
            Some(t) => StmtKind::Relation(built(
                if v >= 0 { CKind::Ccw } else { CKind::Cw },
                t.iter().map(|&i| Some(Arg::Ref(Ref::new(entity_name(EntRef::point(i)))))).collect(),
            )),
            // a key that is not a triple of points has no name to travel under; it is kept
            // verbatim so a document never silently loses one
            None => StmtKind::Branch(crate::syntax::Branch { key: key.clone(), value: v }),
        });
    }
    crate::syntax::render(&mut p);
    p
}

pub(crate) fn lift_decl(sk: &Sketch, e: EntRef) -> Decl {
    let kids = sk.children(e);
    let mut children: Vec<Vec<Kid>> = Vec::new();
    let mut taken = 0usize;
    for (_, field) in e.kind.fields() {
        match field {
            Field::Child => {
                children.push(
                    kids.get(taken)
                        .map(|&k| vec![Kid::Ref(Ref::new(entity_name(k)))])
                        .unwrap_or_default(),
                );
                taken += 1;
            }
            Field::List => {
                children
                    .push(kids[taken..].iter().map(|&k| Kid::Ref(Ref::new(entity_name(k)))).collect());
                taken = kids.len();
            }
            Field::Scalar => {}
        }
    }
    let seed: Vec<f64> =
        sk.own_params(e).iter().map(|&p| sk.params[p as usize].value).collect();
    // a knot vector prints only when it is not the one a control polygon of that length would
    // get anyway: it is document data, and most of it says nothing
    let knots = match e.kind {
        EntKind::Spline => {
            let u = &sk.splines[e.i()].knots;
            let d = curve::clamped_uniform(sk.splines[e.i()].ctrl.len());
            (u.len() != d.len() || u.iter().zip(&d).any(|(a, b)| a != b)).then(|| u.clone())
        }
        _ => None,
    };
    Decl {
        kind: e.kind,
        name: DeclName::Written(Name::new(entity_name(e))),
        children,
        seed_text: vec![None; seed.len()],
        seed_spans: vec![Span::default(); seed.len()],
        hint_span: None,
        seed,
        curve: (e.kind == EntKind::Curve).then(|| lift_curve(sk, e.i())),
        computed: None,
        knots,
        class: sk.class_of(e),
        class_span: Span::default(),
        seed_at: None,
        seed_names: Vec::new(),
        attitude: lift_attitude(sk, e),
        membership: lift_plane(sk, e),
        list_span: Span::default(),
    }
}

/// A curve as a statement spells it: an instance written in place — the component, what it was
/// given, the swept formal at the home — and the point, over the interval.  The component's
/// own text is not in the sketch, so a lifted program parses only beside it.
fn lift_curve(sk: &Sketch, i: usize) -> crate::syntax::CurveSpec {
    use crate::syntax::{CurveSpec, CurveTarget, InstArg, InstVal, Instance};
    let cv = &sk.curves[i];
    let def = &sk.curve_defs[cv.def as usize];
    let arg = |n: &str, v: InstVal| InstArg { label: Some(Name::new(n)), value: v, span: Span::default() };
    let mut args: Vec<InstArg> = def
        .formals
        .iter()
        .zip(&cv.args)
        .map(|((n, _), a)| arg(n, InstVal::Ref(Ref::new(entity_name(*a)))))
        .collect();
    args.extend(
        def.values.iter().zip(&cv.values).map(|(n, v)| arg(n, InstVal::Expr(crate::syntax::num(*v)))),
    );
    if let crate::model::Home::At(u) = cv.home {
        args.push(arg(&def.param, InstVal::Expr(crate::syntax::num(u))));
    }
    CurveSpec {
        target: CurveTarget::Anon(
            Instance {
                name: Name::new("#c"),
                component: Name::new(def.component.clone()),
                args,
                span: Span::default(),
                membership: Default::default(),
                class: Default::default(),
            },
            Ref::new(def.port.clone()),
        ),
        swept: Name::new(def.param.clone()),
        domain: (crate::syntax::num(cv.domain.0), crate::syntax::num(cv.domain.1)),
        of: None,
    }
}

/// A plane's attitude as a statement spells it: nothing for the page's own basis, the basis
/// itself otherwise.  A lifted plane never says `from` — the sketch holds the resolved basis
/// and not the construction it came from.
fn lift_attitude(sk: &Sketch, e: EntRef) -> Attitude {
    if e.kind != EntKind::Plane {
        return Attitude::Page;
    }
    let b = sk.planes[e.i()].basis;
    let page = crate::plane::Basis::page();
    let same = |a: [f64; 3], c: [f64; 3]| (0..3).all(|i| (a[i] - c[i]).abs() < 1e-12);
    if same(b.u, page.u) && same(b.v, page.v) {
        return Attitude::Page;
    }
    let dim = |x: f64| Arg::Dim { text: num(x), span: Span::default() };
    Attitude::Basis {
        u: [dim(b.u[0]), dim(b.u[1]), dim(b.u[2])],
        v: [dim(b.v[0]), dim(b.v[1]), dim(b.v[2])],
    }
}

/// The plane an entity's points are all on, when they are all on one — the clause its
/// statement writes.  A point with none, or a line whose ends are on two planes (which no one
/// statement can say), lifts without one.
pub(crate) fn lift_plane(sk: &Sketch, e: EntRef) -> crate::syntax::Membership {
    match plane_of_entity(sk, e) {
        Some(p) => crate::syntax::Membership::lifted(Ref::new(entity_name(EntRef::plane(p)))),
        None => Default::default(),
    }
}

/// The one plane every point of an entity is on, or `None` — for a point, its own.
pub(crate) fn plane_of_entity(sk: &Sketch, e: EntRef) -> Option<usize> {
    plane_of_entity_by(sk, e, |p| sk.plane_of(p))
}

/// The same walk over any reading of where a point is — membership here, and the overview's
/// `view_of` (which also reads a datum's own points in the plane they place) — so the rule
/// "every point it is made of agrees" is written once.
pub(crate) fn plane_of_entity_by(
    sk: &Sketch,
    e: EntRef,
    at: impl Fn(usize) -> Option<usize>,
) -> Option<usize> {
    // a point answers for itself, and a datum or a curve has no points of its own to answer
    // for; everything else is its children, walked without collecting them twice
    if !e.kind.bears_points() {
        return None;
    }
    if e.kind == EntKind::Point {
        return at(e.i());
    }
    let kids = sk.children(e);
    let first = at(kids.first()?.i())?;
    kids.iter().all(|k| at(k.i()) == Some(first)).then_some(first)
}

/// A `ground` or a `fix` statement, built: `ground p` for a point, `fix c.r` for one of an
/// entity's own numbers.  What `to_program` writes for every held parameter and what
/// `edit::reconcile` appends when the app holds one.
pub(crate) fn lift_gauge(name: &str, field: Option<&str>) -> Relation {
    match field {
        None => built(CKind::Ground, vec![Some(Arg::Ref(Ref::new(name.to_string())))]),
        Some(f) => built(CKind::Fix, vec![Some(Arg::Ref(Ref::field(name.to_string(), f)))]),
    }
}

/// A relation somebody built rather than wrote: the kind and its arguments, and nothing else.
fn built(kind: CKind, args: Vec<Option<Arg>>) -> Relation {
    Relation {
        kind,
        args,
        place: None,
        place_span: Span::default(),
        poly: None,
        claim: false,
        class: Default::default(),
        class_span: Span::default(),
    }
}

pub(crate) fn lift_relation(sk: &Sketch, c: &Constraint) -> Relation {
    let spec = c.kind.spec();
    let mut args: Vec<Option<Arg>> = Vec::with_capacity(spec.len());
    for (i, (_, kind)) in spec.iter().enumerate() {
        args.push(lift_arg(sk, *kind, &c.args[i]));
    }
    Relation {
        kind: c.kind,
        args,
        place: sk.placements.get(&c.id).copied(),
        place_span: Span::default(),
        poly: None,
        claim: c.claim,
        class: c.class.clone(),
        class_span: Span::default(),
    }
}

fn lift_arg(sk: &Sketch, kind: SpecKind, a: &CArg) -> Option<Arg> {
    Some(match a {
        CArg::Ent(e) => Arg::Ref(Ref::new(entity_name(*e))),
        // a hidden unknown travels as the number it holds, and as `==` when it was pinned: a fit
        // chose it, and a document that came back with it free would have degrees of freedom
        // nobody drew
        CArg::Param(i) => Arg::Seed {
            value: sk.params[*i as usize].value,
            pinned: sk.params[*i as usize].fixed,
        },
        CArg::Seed { value, pinned } => Arg::Seed { value: *value, pinned: *pinned },
        // a dimension is written as it was written: `h = w / 2` and `3 1/8` each tell a reader
        // what 40 and 3.125 do not
        CArg::Expr(e) => Arg::Dim { text: e.text.clone(), span: Span::default() },
        CArg::Num(v) if kind.is_dimension() => {
            Arg::Dim { text: num(expr::to_user_units(kind, *v)), span: Span::default() }
        }
        CArg::Num(v) => Arg::Num(*v),
        CArg::Int(v) => Arg::Int(*v),
        CArg::Bool(b) => Arg::Bool(*b),
        CArg::Str(s) => Arg::Word(s.clone()),
    })
}

/* -- text ------------------------------------------------------------------------- */

/// A sketch as a program.  The counterpart of `io::dumps`.
pub fn dumps(sk: &Sketch) -> String {
    to_program(sk).text().to_string()
}
