//! Solvent: the language a sketch is written in.
//!
//! Tokens, spans, the syntax tree and the printer.  The parser joins them here too, for the reason
//! `io.rs` holds `to_json` and `from_json` together: the round trip *is* the contract, and two
//! halves of one agreement are best read side by side.
//!
//! **Nothing here is written per constraint type.**  A statement's name is the snake_case of
//! `CKind::name()`, its arguments follow `CKind::spec()`, and a trailing `Length`/`Angle` slot
//! prints after `==` — the same bargain `report::registry_json` already strikes with the Python
//! and TypeScript bindings, so a new constraint type appears in the language with nothing to
//! change.  Entity declarations are named by `EntKind::fields`, which is the document's own table.
//!
//! **The one exception is `joint_relation`**, and it is the exception that proves the rule: a
//! chain's `tangent` is a *drafting* word, not a constraint name, and which regular form it means
//! — `TangentArcLine` at an end, collinearity between two straight runs — depends on the pair of
//! kinds it stands between.  No registry lookup can answer that, because the answer is which
//! constraint to state rather than how to spell one.  Everything else the chain grammar admits is
//! derived: `prefix_kind` and `infix_kind` ask `CKind::spec()` for eligibility, so a new unary or
//! binary constraint joins the language with nothing here to edit.
//!
//! Two spellings carry the whole hint/constraint classification the language rests on:
//!
//! * `=` **seeds**.  `at (0, 0)`, `r: 25`, `t = 0.37` are inert — deleting every one of them
//!   changes no solution set, only which solution is found, so a solve may write them back.
//! * `==` **constrains**.  `== 80` and `t == 0.37` state something, and a solve must never
//!   rewrite one.  A pinned curve parameter is `==` precisely because it changes the solution
//!   set: without the pin, a curve fitted through m points keeps m degrees of freedom.
//!
//! That distinction is lexical, which is what makes "may a solve write this number?" a test
//! rather than an analysis — see `program::commit_seeds`.

use crate::constraints::{CKind, SpecKind};
use crate::model::{EntKind, EntRef, Field};

/// How long a program may be.  A document is untrusted input and `wasm32-unknown-unknown` aborts
/// rather than unwinding, so the size is checked here rather than left to an allocator.
pub const MAX_TEXT: usize = 1 << 20;

/// How many statements one may hold.
pub const MAX_STMTS: usize = 100_000;

/// A half-open byte range into the program text.
///
/// Bytes, not characters: the front end slices the same `&str` the core parsed, and a UTF-8
/// boundary is the only thing the two ever have to agree about.  Line and column are *not* stored
/// — `line_col` computes them on demand, so there is nothing to keep in step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Span {
    pub lo: u32,
    pub hi: u32,
}

impl Span {
    pub fn new(lo: usize, hi: usize) -> Span {
        Span { lo: lo as u32, hi: hi as u32 }
    }

    pub fn slice(self, text: &str) -> &str {
        text.get(self.lo as usize..self.hi as usize).unwrap_or("")
    }

    pub fn contains(self, off: u32) -> bool {
        self.lo <= off && off < self.hi
    }

    pub fn len(self) -> u32 {
        self.hi.saturating_sub(self.lo)
    }

    pub fn is_empty(self) -> bool {
        self.len() == 0
    }
}

/// 1-based line and column at a byte offset.  Columns count characters, not bytes, because that
/// is what an editor's caret does.
pub fn line_col(text: &str, off: u32) -> (u32, u32) {
    let off = (off as usize).min(text.len());
    let head = &text[..off];
    let line = 1 + head.bytes().filter(|&b| b == b'\n').count() as u32;
    let bol = head.rfind('\n').map(|i| i + 1).unwrap_or(0);
    (line, 1 + text[bol..off].chars().count() as u32)
}

/// A name as it was written, and where.
#[derive(Clone, Debug, PartialEq)]
pub struct Name {
    pub text: String,
    pub span: Span,
}

impl Name {
    pub fn new(text: impl Into<String>) -> Name {
        Name { text: text.into(), span: Span::default() }
    }
}

/// A statement's identity, minted by whoever built it and **preserved by every edit**.  Not a
/// position: inserting a statement above must not renumber one a caller is holding on to, since a
/// selection outlives the elaboration that resolved it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct StmtId(pub u32);

/// A whole program.  It owns its text, and every `Span` in it indexes that text — which is why
/// `render` writes both at once rather than leaving two things to keep in step.
#[derive(Clone, Debug, Default)]
pub struct Program {
    text: String,
    /// One, anonymous, in the flat subset; `component` is a parser addition rather than a change
    /// of shape.
    pub components: Vec<Component>,
    /// The curve families the program defines.  Program-level like a component, and for the same
    /// reason: a family is a *kind of curve*, not a curve, and several drawings may be written
    /// over one.
    pub curves: Vec<CurveFamily>,
    next_stmt: u32,
}

/// `curve involute(c: circle, phase: Angle)(u) over (0, 90) = ( xexpr, yexpr )`
///
/// The expressions are kept as *text* and compiled by `program::elaborate`, exactly as a
/// dimension's is: the little language they are written in is `expr.rs`'s, and reading it a
/// second time here would be a second copy of rules like the one that makes `3 1/8` a number.
#[derive(Clone, Debug)]
pub struct CurveFamily {
    pub name: Name,
    /// The entities the curve is written over, and the numbers it takes.
    pub formals: Vec<Formal>,
    /// What it runs on.
    pub param: Name,
    /// The interval it is drawn over unless an instance narrows it.
    pub domain: Option<(String, String)>,
    pub body: FamilyBody,
    pub span: Span,
}

/// What follows a family's `=`: a pair of expressions, or `trace p [from (expr)] where { … }` —
/// a point and the constraints that force it, the curve then being wherever they put the point
/// as the parameter runs.  `from` names the parameter value evaluation is anchored at: the one
/// place the block's orientation predicates are read, chosen so they read unambiguously.  The
/// block's statements are ordinary statements; what may appear in one is
/// `program::compile_trace`'s question, not the parser's.
#[derive(Clone, Debug)]
pub enum FamilyBody {
    Exprs { x: String, y: String, xspan: Span, yspan: Span },
    Trace { point: Name, home: Option<(String, Span)>, body: Vec<Stmt> },
}

#[derive(Clone, Debug, Default)]
pub struct Component {
    pub name: Option<Name>,
    pub formals: Vec<Formal>,
    pub body: Vec<Stmt>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct Formal {
    pub name: Name,
    pub ty: Ty,
    pub span: Span,
}

/// The spec's §3.1 value types, plus the entity kinds this model actually has.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ty {
    Int,
    Scalar,
    Length,
    Angle,
    Ent(EntKind),
}

impl Ty {
    /// The word a type is written as.  One table: a formal's type, a curve family's, and the
    /// colouring of the word are the same question, and a second copy of the list would be a
    /// second answer the moment a type is added.
    pub fn parse(s: &str) -> Option<Ty> {
        Some(match s {
            "Int" => Ty::Int,
            "Scalar" => Ty::Scalar,
            "Length" => Ty::Length,
            "Angle" => Ty::Angle,
            other => Ty::Ent(EntKind::parse(&other.to_lowercase())?),
        })
    }
}

#[derive(Clone, Debug)]
pub struct Stmt {
    pub id: StmtId,
    pub kind: StmtKind,
    pub span: Span,
    /// How this statement's text is spelled — whole line, or one part of a chain's.  The parser
    /// knows it while it is desugaring, so it is recorded rather than sniffed back out of the
    /// characters later.
    pub chained: Chained,
}

/// What a statement's text *is*, where a chain (spec §6.6) wrote it.
///
/// A chain puts several statements on one line, so the whole-line splice that deletes an
/// ordinary statement would take its neighbours with it.  Which word to splice is a question the
/// parser already answered when it desugared, and `edit::doom_splice` matches on the answer —
/// as against re-lexing the source, which would have to rest on "a longhand relation always
/// carries a `(`", an invariant nothing states and a qualified joint would quietly break.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Chained {
    /// A statement of its own, occupying its whole line.
    #[default]
    No,
    /// The declaration a link makes — a fragment of a line others share, which is why deleting
    /// a chained entity is refused rather than half-done.
    Link,
    /// A unary word standing before a link: `horizontal line …`.
    Prefix,
    /// A word standing between two links: `… tangent …`.
    Joint,
    /// The joint before `close`, which seals a loop.
    Close,
}

#[derive(Clone, Debug)]
pub enum StmtKind {
    Decl(Decl),
    Relation(Relation),
    Gauge(Gauge),
    /// A recorded root choice.  Spec §9.6's orientation predicates are exactly this: they
    /// contribute no equations and select among the discrete solution components.
    Orient(Orient),
    /// `t: Tooth(root, tip, slot: 360 / N)` — a component, elaborated in place.
    Instance(Instance),
    /// `port lead: Point` declares one and exports it; `port hub = f0` exports one that exists.
    /// A port is *a name on the boundary for an interior entity, and nothing more* (spec §7):
    /// binding costs no residual because it is aliasing, not constraint.
    Port(Port),
    /// `param R = m * N / 2` — a number worked out while elaborating, never an unknown.
    Param(ParamDecl),
    /// `repeat`, `cycle`, `ring` — see `Block`.
    Block(Block),
}

#[derive(Clone, Debug)]
pub struct Instance {
    pub name: Name,
    pub component: Name,
    pub args: Vec<InstArg>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct InstArg {
    pub label: Option<Name>,
    /// An entity argument binds by aliasing; a value argument is a number worked out here.
    pub value: InstVal,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub enum InstVal {
    Ref(Ref),
    /// An expression over the enclosing component's own parameters, evaluated while elaborating.
    Expr(String),
}

#[derive(Clone, Debug)]
pub struct Port {
    pub name: Name,
    /// `port x: Point` — declare one of this kind and export it.
    pub declare: Option<EntKind>,
    /// `port x = y` — export one that already exists.
    pub alias: Option<Ref>,
}

#[derive(Clone, Debug)]
pub struct ParamDecl {
    pub name: Name,
    pub text: String,
    pub span: Span,
}

/// Repetition.  Three constructs and three meanings (spec §12).
#[derive(Clone, Debug)]
pub struct Block {
    pub kind: BlockKind,
    /// How many, as an expression over the enclosing parameters.
    pub count: String,
    /// `ring N about center` — the axis every instance turns about.
    pub about: Option<Ref>,
    /// `as i` — the index, available to every expression inside.
    pub binder: Option<Name>,
    pub body: Vec<Stmt>,
    pub span: Span,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockKind {
    /// N copies, and no relation between them.  `next`/`prev` are not in scope.
    Repeat,
    /// N copies that close: `next` is instance (i+1) mod N, `prev` is (i-1) mod N.
    Cycle,
    /// A cycle whose instances are claimed to be congruent about an axis.
    Ring,
}

/// `point p0 at (0, 0)`, `circle c0(center: p2, r: 25)`, `spline s0(p3, p4, p5, p6) knots [...]`.
#[derive(Clone, Debug)]
pub struct Decl {
    pub kind: EntKind,
    pub name: Name,
    /// One per `Child`/`List` field of `EntKind::fields`, in that order; a `List` field holds as
    /// many as were written.
    pub children: Vec<Vec<Ref>>,
    /// One per `Scalar` field — the entity's seed, and hint-class.
    pub seed: Vec<f64>,
    /// The same, as written, where it was written as an expression over the enclosing component's
    /// parameters (`circle root(center: c, r: Rr)`).  Worked out during expansion and `None` from
    /// then on, so a printed program only ever carries numbers.
    pub seed_text: Vec<Option<String>>,
    /// Where each seed sits in the source, so a solve can write one back **without reprinting
    /// the statement around it**.  That is the whole difference between a program that is the
    /// document and a program that is a view of one: a drag rewrites six characters and leaves
    /// every comment, every blank line and every hand-written component exactly as it was.
    ///
    /// Empty for a declaration that was built rather than parsed — there is no text to splice.
    pub seed_spans: Vec<Span>,
    /// Document data no solve moves, so not a seed and never written back.
    pub knots: Option<Vec<f64>>,
    /// A curve instance: the family it belongs to.  `None` for every other kind.
    pub def: Option<Name>,
    /// The numbers a curve instance is given, as written.
    pub values: Vec<(Name, String)>,
    /// The interval a curve instance is drawn over, as written.
    pub domain: Option<(String, String)>,
    pub construction: bool,
    /// A seed named *geometrically* rather than by coordinates: `at t`, `at c.center`,
    /// `at c bearing (u + phase)`.  What it may name is the elaborator's question.
    pub seed_at: Option<AtRef>,
}

/// `at c bearing (u + phase)` — a place given as geometry: at a point, or at the edge of a
/// circle at a bearing from the page's x-axis.
#[derive(Clone, Debug)]
pub struct AtRef {
    pub what: Ref,
    pub bearing: Option<(String, Span)>,
}

/// A constraint statement: `distance(p0, p1) == 80 at (12, -4)`.
#[derive(Clone, Debug)]
pub struct Relation {
    pub kind: CKind,
    /// One per `CKind::spec()` slot; `None` where the source left an inferred slot out.
    pub args: Vec<Option<Arg>>,
    /// Where the callout was dragged to, if anywhere.  A seed: inert, and written back.
    pub place: Option<(f64, f64)>,
}

/// One argument as written.
#[derive(Clone, Debug)]
pub enum Arg {
    Ref(Ref),
    Num(f64),
    Int(i64),
    Bool(bool),
    /// A bare identifier in a `Str` slot — `at: start` — or a quoted string.
    Word(String),
    /// Everything after the trailing `==`, verbatim, for `expr::parse`.  Not tokenized here: the
    /// dimension sub-language is `expr.rs`'s, and a second tokenizer would be a second copy of
    /// rules like the one that makes `3 1/8` a number and `31/2` a division.
    Dim { text: String, span: Span },
    /// A slot the constraint owns.  `pinned` is the `==` spelling: somebody said where along, and
    /// the solver is not to argue.
    Seed { value: f64, pinned: bool },
    /// The same, written over the parameters in scope — `u = u0` inside a component.  Worked out
    /// during expansion and a plain `Seed` from then on.
    SeedExpr { text: String, pinned: bool, span: Span },
}

/// `p0`, `c0.r`.  `path` is empty throughout the flat subset and is here from the start so that
/// `t.lead` and `name[k]` are a parser addition rather than a change of shape.
#[derive(Clone, Debug, PartialEq)]
pub struct Ref {
    pub root: Name,
    pub path: Vec<Seg>,
    pub span: Span,
}

impl Ref {
    pub fn new(name: impl Into<String>) -> Ref {
        Ref { root: Name::new(name), path: Vec::new(), span: Span::default() }
    }

    pub fn field(name: impl Into<String>, f: &str) -> Ref {
        Ref {
            root: Name::new(name),
            path: vec![Seg::Field(Name::new(f))],
            span: Span::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Seg {
    Field(Name),
    /// `p[i + 1]` — *which copy* of a repeated statement, as written.  Held as text because the
    /// index is an expression over the counts and binders in scope, and those are not known until
    /// the block is expanded; `flatten` works it out there.  Spec §12.5.
    Index(String),
}

/// `ground(p0)` pins both of a point's coordinates; `fix(c0.r)` pins one scalar.  Deliberately
/// narrow: exactly what the document can already store, and no sugar that would not round-trip.
#[derive(Clone, Debug)]
pub enum Gauge {
    Ground(Ref),
    Fix(Ref),
}

/// `ccw(p0, p3, p7)` — a recorded root choice, named by its points rather than by their indices.
#[derive(Clone, Debug)]
pub struct Orient {
    pub ccw: bool,
    pub pts: Vec<Ref>,
    /// A key `decompose::branch_key_points` could not read as a triple, kept verbatim so a
    /// document never silently loses one.
    pub raw: Option<(String, i32)>,
}

/* -- names ------------------------------------------------------------------------- */

/// What the language calls an entity: the first letter of its kind and its index — `p0`, `l1`,
/// `c0`, `a2`, `s0`, `e0`.  The six initials are distinct, which is the whole of why this works.
///
/// Lowercase, unlike `io::entity_name`'s `P0`: that one labels a thing on a drawing, this one is
/// an identifier in a program, and the two are read in different places.
pub fn entity_name(e: EntRef) -> String {
    format!("{}{}", kind_initial(e.kind), e.idx)
}

pub fn kind_initial(k: EntKind) -> char {
    k.as_str().chars().next().expect("every kind name has a letter")
}

/// `PointOnLine` → `point_on_line`.  A run of capitals stays together, so `K33`-shaped names do
/// not come apart, though none of the 32 has one.
pub fn snake(name: &str) -> String {
    let mut out = String::with_capacity(name.len() + 4);
    let cs: Vec<char> = name.chars().collect();
    for (i, &c) in cs.iter().enumerate() {
        if c.is_ascii_uppercase() {
            let starts_word = i > 0
                && (!cs[i - 1].is_ascii_uppercase()
                    || cs.get(i + 1).is_some_and(|n| n.is_ascii_lowercase()));
            if starts_word {
                out.push('_');
            }
            out.push(c.to_ascii_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}

/// `point_on_line` → `PointOnLine`, the inverse of `snake` on every name the registry holds —
/// which `tests/syntax.rs` checks for all 32 rather than assuming.
pub fn camel(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    let mut up = true;
    for c in name.chars() {
        if c == '_' {
            up = true;
        } else if up {
            out.extend(c.to_uppercase());
            up = false;
        } else {
            out.push(c);
        }
    }
    out
}

/// A number as text: shortest round-trip, which is what a document needs.
///
/// Never `json::fmt_g` — that rounds for display, and a value rounded on the way out is a drawing
/// that moved because somebody looked at it.
pub fn num(v: f64) -> String {
    if !v.is_finite() {
        // a document should never hold one, and printing `inf` would produce a program that does
        // not parse; 0 is wrong in a way somebody will notice, which is the point
        return "0".to_string();
    }
    format!("{v}")
}

/* -- the program ------------------------------------------------------------------- */

impl Program {
    pub fn new() -> Program {
        Program {
            text: String::new(),
            components: vec![Component::default()],
            curves: Vec::new(),
            next_stmt: 0,
        }
    }

    /// The text this program was last rendered or parsed from.  Every `Span` indexes it.
    pub fn text(&self) -> &str {
        &self.text
    }

    /// The root component — the one the drawing is.  The flat subset has exactly one.
    pub fn root(&self) -> &Component {
        self.components.last().expect("a program has a root")
    }

    pub fn root_mut(&mut self) -> &mut Component {
        self.components.last_mut().expect("a program has a root")
    }

    /// A fresh statement identity.  Monotonic, and never reused, so a caller holding one can only
    /// find the statement it named or nothing at all.
    pub fn mint(&mut self) -> StmtId {
        self.next_stmt += 1;
        StmtId(self.next_stmt)
    }

    pub fn push(&mut self, kind: StmtKind) -> StmtId {
        let id = self.mint();
        let st = Stmt { id, kind, span: Span::default(), chained: Chained::No };
        self.root_mut().body.push(st);
        id
    }

    /// The statement with this id, blocks and all.
    ///
    /// A direct walk rather than `stmts().find(…)`: this is asked once per entity by
    /// `edit::commit_seeds` and once per entity again by `reconcile`, and `stmts` builds a `Vec`
    /// of the whole program to hand back.  Here it short-circuits and allocates nothing.
    pub fn stmt(&self, id: StmtId) -> Option<&Stmt> {
        fn find(body: &[Stmt], id: StmtId) -> Option<&Stmt> {
            for st in body.iter() {
                if st.id == id {
                    return Some(st);
                }
                if let StmtKind::Block(b) = &st.kind {
                    if let Some(hit) = find(&b.body, id) {
                        return Some(hit);
                    }
                }
            }
            None
        }
        self.components.iter().find_map(|c| find(&c.body, id))
    }

    /// The innermost statement covering a byte offset — a caret, turned into what it is written
    /// on.  A linear scan: this runs on a click, never on a frame.
    pub fn at_offset(&self, off: u32) -> Option<&Stmt> {
        self.stmts().filter(|s| s.span.contains(off)).min_by_key(|s| s.span.len())
    }

    /// Every statement in the program, blocks and all.
    ///
    /// **Including a block's body**, because a statement inside a `cycle` is a statement: it is
    /// what a span points at, what a caret lands on and what an expanded entity names.  Stopping
    /// at the block would make a gear's hundred and twenty points come from nothing findable.
    /// Whether a statement is one the *root* may splice on its own is a different question, asked
    /// against `root().body` where it belongs.
    pub fn stmts(&self) -> impl Iterator<Item = &Stmt> {
        fn walk<'a>(st: &'a Stmt, out: &mut Vec<&'a Stmt>) {
            out.push(st);
            if let StmtKind::Block(b) = &st.kind {
                for inner in b.body.iter() {
                    walk(inner, out);
                }
            }
        }
        let mut out = Vec::new();
        for c in self.components.iter() {
            for st in c.body.iter() {
                walk(st, &mut out);
            }
        }
        out.into_iter()
    }
}

/* -- printing ---------------------------------------------------------------------- */

/// How wide the entity keyword column is: `ellipse`, the longest of the six, and a constant — so
/// the aligned look never makes one edit reflow the whole file.
const KW: usize = 7;

/// Write the program out and stamp every span in it.
///
/// Both at once, because a `Program` owns its text and its spans index that text: making them
/// separately would be two things to keep in step, and one of them would eventually be wrong.
/// The returned text is the program's own — the caller does not get a copy to diverge from.
pub fn render(p: &mut Program) -> &str {
    let mut out = String::new();
    let mut spans: Vec<(usize, usize, Span)> = Vec::new();
    let comps = p.components.len();
    for (ci, comp) in p.components.iter().enumerate() {
        let lo = out.len();
        // declarations and constraints in two runs with a rule between them, when a component
        // holds both: it is how a reader expects to meet a drawing — what there is, then what is
        // true of it
        let mut said_decl = false;
        for (si, st) in comp.body.iter().enumerate() {
            let is_decl = matches!(st.kind, StmtKind::Decl(_));
            if said_decl && !is_decl {
                out.push('\n');
                said_decl = false;
            }
            said_decl |= is_decl;
            let s0 = out.len();
            write_stmt(&mut out, &st.kind);
            spans.push((ci, si, Span::new(s0, out.len())));
            out.push('\n');
        }
        let mut span = Span::new(lo, out.len());
        if comps > 1 {
            span = Span::new(lo, out.len());
        }
        spans.push((ci, usize::MAX, span));
    }
    for (ci, si, span) in spans {
        if si == usize::MAX {
            p.components[ci].span = span;
        } else {
            p.components[ci].body[si].span = span;
        }
    }
    p.text = out;
    &p.text
}

/// One statement, as text.  The one place a statement is written down, so an edit that appends
/// one and a printer that prints a whole program cannot disagree about how it reads.
pub fn write_stmt_to(out: &mut String, k: &StmtKind) {
    write_stmt(out, k)
}

fn write_stmt(out: &mut String, k: &StmtKind) {
    match k {
        StmtKind::Decl(d) => write_decl(out, d),
        StmtKind::Relation(r) => write_relation(out, r),
        StmtKind::Gauge(Gauge::Ground(r)) => {
            out.push_str("ground(");
            write_ref(out, r);
            out.push(')');
        }
        StmtKind::Gauge(Gauge::Fix(r)) => {
            out.push_str("fix(");
            write_ref(out, r);
            out.push(')');
        }
        StmtKind::Orient(o) => write_orient(out, o),
        StmtKind::Instance(i) => {
            out.push_str(&format!("{}: {}(", i.name.text, i.component.text));
            let parts: Vec<String> = i
                .args
                .iter()
                .map(|a| {
                    let mut s = String::new();
                    if let Some(l) = &a.label {
                        s.push_str(&l.text);
                        s.push_str(": ");
                    }
                    match &a.value {
                        InstVal::Ref(r) => write_ref(&mut s, r),
                        InstVal::Expr(t) => s.push_str(t),
                    }
                    s
                })
                .collect();
            out.push_str(&parts.join(", "));
            out.push(')');
        }
        StmtKind::Port(p) => {
            out.push_str(&format!("port {}", p.name.text));
            if let Some(k) = p.declare {
                out.push_str(&format!(": {}", camel(k.as_str())));
            } else if let Some(r) = &p.alias {
                out.push_str(" = ");
                write_ref(out, r);
            }
        }
        StmtKind::Param(p) => out.push_str(&format!("param {} = {}", p.name.text, p.text)),
        StmtKind::Block(b) => {
            out.push_str(match b.kind {
                BlockKind::Repeat => "repeat ",
                BlockKind::Cycle => "cycle ",
                BlockKind::Ring => "ring ",
            });
            out.push_str(&b.count);
            if let Some(a) = &b.about {
                out.push_str(" about ");
                write_ref(out, a);
            }
            if let Some(i) = &b.binder {
                out.push_str(&format!(" as {}", i.text));
            }
            out.push_str(" {\n");
            for st in &b.body {
                out.push_str("  ");
                write_stmt(out, &st.kind);
                out.push('\n');
            }
            out.push('}');
        }
    }
}

/// Whether a declaration labels its children.
///
/// A line's two ends and a control polygon read better bare — `line l0(p0, p1)` says everything,
/// and `p1: p0` would put a field name that looks like a point name in front of a point.
/// Everything else is labelled, because `arc a0(center: p4, start: p5, end: p6)` tells a reader
/// which is which and three bare point names do not.  The parser requires labels nowhere.
fn labels_children(k: EntKind) -> bool {
    !matches!(k, EntKind::Line | EntKind::Spline)
}

fn write_decl(out: &mut String, d: &Decl) {
    let kw = d.kind.as_str();
    out.push_str(kw);
    for _ in kw.len()..KW {
        out.push(' ');
    }
    out.push(' ');
    out.push_str(&d.name.text);

    let label = labels_children(d.kind);
    let mut parts: Vec<String> = Vec::new();
    let mut child = 0usize;
    let mut scalar = 0usize;
    for (name, field) in d.kind.fields() {
        match field {
            Field::Child | Field::List => {
                let refs = d.children.get(child).cloned().unwrap_or_default();
                child += 1;
                for r in &refs {
                    let mut s = String::new();
                    if label && *field == Field::Child {
                        s.push_str(name);
                        s.push_str(": ");
                    }
                    write_ref(&mut s, r);
                    parts.push(s);
                }
            }
            Field::Scalar => {
                // a point's coordinates are its `at` clause, not constructor arguments: a point
                // *is* a place, and `point p0(x: 0, y: 0)` says it twice
                if d.kind == EntKind::Point {
                    scalar += 1;
                    continue;
                }
                let v = d.seed.get(scalar).copied().unwrap_or(0.0);
                scalar += 1;
                parts.push(format!("{name}: {}", num(v)));
            }
        }
    }
    if !parts.is_empty() {
        out.push('(');
        out.push_str(&parts.join(", "));
        out.push(')');
    }
    if d.kind == EntKind::Point {
        let x = d.seed.first().copied().unwrap_or(0.0);
        let y = d.seed.get(1).copied().unwrap_or(0.0);
        out.push_str(&format!(" at ({}, {})", num(x), num(y)));
    }
    if let Some(u) = &d.knots {
        out.push_str(" knots [");
        out.push_str(&u.iter().map(|&v| num(v)).collect::<Vec<_>>().join(", "));
        out.push(']');
    }
    if d.construction {
        out.push_str(" construction");
    }
}

fn write_relation(out: &mut String, r: &Relation) {
    out.push_str(&snake(r.kind.name()));
    let spec = r.kind.spec();
    // a trailing Length or Angle is what the statement *states*, and goes after `==`
    let tail = spec.len().checked_sub(1).filter(|&i| spec[i].1.is_dimension());
    let mut parts: Vec<String> = Vec::new();
    for (i, (name, sk)) in spec.iter().enumerate() {
        if Some(i) == tail {
            continue;
        }
        let Some(a) = r.args.get(i).and_then(|a| a.as_ref()) else { continue };
        parts.push(write_arg(name, *sk, a));
    }
    out.push('(');
    out.push_str(&parts.join(", "));
    out.push(')');
    if let Some(i) = tail {
        if let Some(a) = r.args.get(i).and_then(|a| a.as_ref()) {
            out.push_str(" == ");
            out.push_str(&dim_text(a));
        }
    }
    if let Some((t, rr)) = r.place {
        out.push_str(&format!(" at ({}, {})", num(t), num(rr)));
    }
}

/// One argument.  Entity slots are positional — the statement's name says what they are — and
/// everything else is labelled, because `side: 1` says what `1` does not.
fn write_arg(name: &str, sk: SpecKind, a: &Arg) -> String {
    match a {
        Arg::Ref(r) => {
            let mut s = String::new();
            write_ref(&mut s, r);
            s
        }
        Arg::Seed { value, pinned } => {
            // `==` for a pin and `=` for a seed: the one lexical fact `commit_seeds` reads
            format!("{name} {} {}", if *pinned { "==" } else { "=" }, num(*value))
        }
        Arg::Num(v) if sk == SpecKind::Angle => format!("{name}: {}", num(v.to_degrees())),
        Arg::Num(v) => format!("{name}: {}", num(*v)),
        Arg::Int(v) => format!("{name}: {v}"),
        Arg::Bool(b) => format!("{name}: {b}"),
        Arg::Word(w) => format!("{name}: {}", word(w)),
        Arg::Dim { text, .. } => format!("{name}: {text}"),
        Arg::SeedExpr { text, pinned, .. } => {
            format!("{name} {} {text}", if *pinned { "==" } else { "=" })
        }
    }
}

/// A `Str` argument bare when it is an identifier — `at: start` — and quoted when it is not, so
/// the parser never has to guess and an empty one is still visible.
fn word(w: &str) -> String {
    let plain = !w.is_empty()
        && w.starts_with(|c: char| c.is_ascii_alphabetic() || c == '_')
        && w.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
    if plain {
        w.to_string()
    } else {
        format!("{w:?}")
    }
}

/// What goes after `==`: the dimension's own text, as written.
fn dim_text(a: &Arg) -> String {
    match a {
        Arg::Dim { text, .. } => text.clone(),
        Arg::Num(v) => num(*v),
        other => write_arg("", SpecKind::Length, other),
    }
}

fn write_orient(out: &mut String, o: &Orient) {
    if let Some((key, v)) = &o.raw {
        out.push_str(&format!("branch({key:?}, {v})"));
        return;
    }
    out.push_str(if o.ccw { "ccw(" } else { "cw(" });
    for (i, r) in o.pts.iter().enumerate() {
        if i > 0 {
            out.push_str(", ");
        }
        write_ref(out, r);
    }
    out.push(')');
}

fn write_ref(out: &mut String, r: &Ref) {
    out.push_str(&r.root.text);
    for seg in &r.path {
        match seg {
            Seg::Field(f) => {
                out.push('.');
                out.push_str(&f.text);
            }
            Seg::Index(t) => out.push_str(&format!("[{t}]")),
        }
    }
}

/* -- lexing ------------------------------------------------------------------------ */

/// What the parser could not make of the text.  A code and a span, so it lands in the same
/// gutter as everything elaboration and the solver have to say — see `program::Diag`, which this
/// becomes.
#[derive(Clone, Debug)]
pub struct SynErr {
    pub span: Span,
    pub message: String,
}

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Ident(String),
    Num(f64),
    Str(String),
    /// `(` `)` `[` `]` `,` `:` `.` `-` `{` `}`
    P(char),
    /// `=` — a seed
    Eq,
    /// `==` — a constraint
    EqEq,
    /// end of a statement: a newline or a `;`
    Nl,
}

struct Lexed {
    toks: Vec<(Tok, Span)>,
    /// Where the comments were.  The parser has no use for them — they are not the program — but
    /// they *are* the document, and `highlight` is the one reader that has to show them.
    comments: Vec<Span>,
}

/// Tokenize.  Errors are collected rather than thrown: one bad character costs one statement, and
/// the rest of the drawing still comes back.
fn lex(src: &str) -> (Lexed, Vec<SynErr>) {
    let b = src.as_bytes();
    let mut toks: Vec<(Tok, Span)> = Vec::new();
    let mut comments: Vec<Span> = Vec::new();
    let mut errs: Vec<SynErr> = Vec::new();
    let mut i = 0usize;
    // A newline ends a statement, but not inside brackets: an argument list may be written across
    // several lines, and a line break there is a separator like any other whitespace.  Braces do
    // *not* count — a body is made of statements, and those still end at their line's end.
    let mut depth = 0i32;
    while i < b.len() {
        let c = b[i] as char;
        let lo = i;
        match c {
            ' ' | '\t' | '\r' => i += 1,
            '\n' => {
                i += 1;
                if depth == 0 {
                    toks.push((Tok::Nl, Span::new(lo, i)));
                }
            }
            ';' => {
                i += 1;
                toks.push((Tok::Nl, Span::new(lo, i)));
            }
            '/' if b.get(i + 1) == Some(&b'/') => {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
                comments.push(Span::new(lo, i));
            }
            '/' if b.get(i + 1) == Some(&b'*') => {
                i += 2;
                while i + 1 < b.len() && !(b[i] == b'*' && b[i + 1] == b'/') {
                    i += 1;
                }
                i = (i + 2).min(b.len());
                comments.push(Span::new(lo, i));
            }
            '=' => {
                if b.get(i + 1) == Some(&b'=') {
                    i += 2;
                    toks.push((Tok::EqEq, Span::new(lo, i)));
                } else {
                    i += 1;
                    toks.push((Tok::Eq, Span::new(lo, i)));
                }
            }
            '"' => {
                i += 1;
                let mut s = String::new();
                while i < b.len() && b[i] != b'"' {
                    if b[i] == b'\\' && i + 1 < b.len() {
                        i += 1;
                        s.push(match b[i] {
                            b'n' => '\n',
                            b't' => '\t',
                            other => other as char,
                        });
                    } else {
                        // walk by character, so a multi-byte one survives
                        let ch = src[i..].chars().next().unwrap_or('"');
                        s.push(ch);
                        i += ch.len_utf8() - 1;
                    }
                    i += 1;
                }
                if i >= b.len() {
                    errs.push(SynErr {
                        span: Span::new(lo, b.len()),
                        message: "a string with no closing quote".to_string(),
                    });
                } else {
                    i += 1;
                }
                toks.push((Tok::Str(s), Span::new(lo, i)));
            }
            '(' | ')' | '[' | ']' | ',' | ':' | '{' | '}' | '-' | '+' => {
                i += 1;
                match c {
                    '(' | '[' => depth += 1,
                    ')' | ']' => depth = (depth - 1).max(0),
                    _ => {}
                }
                toks.push((Tok::P(c), Span::new(lo, i)));
            }
            '.' if !b.get(i + 1).is_some_and(|d| d.is_ascii_digit()) => {
                i += 1;
                toks.push((Tok::P('.'), Span::new(lo, i)));
            }
            c if c.is_ascii_digit() || c == '.' => {
                while i < b.len()
                    && ((b[i] as char).is_ascii_digit()
                        || b[i] == b'.'
                        || b[i] == b'e'
                        || b[i] == b'E'
                        || ((b[i] == b'+' || b[i] == b'-')
                            && matches!(b.get(i - 1), Some(b'e') | Some(b'E'))))
                {
                    i += 1;
                }
                let text = &src[lo..i];
                match text.parse::<f64>() {
                    Ok(v) if v.is_finite() => toks.push((Tok::Num(v), Span::new(lo, i))),
                    _ => errs.push(SynErr {
                        span: Span::new(lo, i),
                        message: format!("`{text}` is not a number"),
                    }),
                }
            }
            c if c.is_alphabetic() || c == '_' => {
                while i < b.len() && {
                    let ch = src[i..].chars().next().unwrap_or(' ');
                    ch.is_alphanumeric() || ch == '_'
                } {
                    i += src[i..].chars().next().map(|c| c.len_utf8()).unwrap_or(1);
                }
                toks.push((Tok::Ident(src[lo..i].to_string()), Span::new(lo, i)));
            }
            // Anything else becomes a token rather than an error.  Most of what lands here is
            // arithmetic — `w / 5`, `2 * a + 5` — inside a dimension, which the parser takes from
            // the source verbatim and never asks the lexer about; erroring on it here would
            // report the one part of a program this file deliberately does not read.  A character
            // that really is out of place is caught where it is *used*, with a span on the
            // statement that wanted something else.
            other => {
                i += src[i..].chars().next().map(|c| c.len_utf8()).unwrap_or(1);
                toks.push((Tok::P(other), Span::new(lo, i)));
            }
        }
    }
    (Lexed { toks, comments }, errs)
}

/* -- colouring --------------------------------------------------------------------- */

/// What a run of the source *is*, for somebody reading it.
///
/// Lexical, and only lexical: this is the parser's own scan, told to keep the comments and to say
/// what each token turned out to be.  A front end colouring a program therefore cannot disagree
/// with the one reading it — a second lexer in TypeScript would be a second language, and would
/// drift on the first thing this one learned (`==` against `=`, a mixed fraction, a block comment).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tint {
    Comment,
    Str,
    Num,
    /// `component`, `param`, `port`, `cycle`, `point`, `over`, `construction` — a word that starts
    /// a statement or shapes one
    Word,
    /// `Angle`, `circle`, `Tooth` — a word in the place a type is written
    Type,
    /// a constraint's name, where the statement is one: `distance`, `ground`, `point_on_circle`
    Relation,
    /// the name a statement gives what it declares, and the binder a block counts by
    Def,
    /// `r:` — a slot named where it is filled
    Label,
    /// `=`, a seed the solver may move
    Seed,
    /// `==`, a claim it may not
    Claim,
}

impl Tint {
    /// The class a front end styles it by.  Named here so the core says what the colours are *of*
    /// and a stylesheet only says what they look like.
    pub fn as_str(self) -> &'static str {
        match self {
            Tint::Comment => "comment",
            Tint::Str => "string",
            Tint::Num => "number",
            Tint::Word => "word",
            Tint::Type => "type",
            Tint::Relation => "relation",
            Tint::Def => "def",
            Tint::Label => "label",
            Tint::Seed => "seed",
            Tint::Claim => "claim",
        }
    }
}

/* The words a statement may open with, where more than one word opens the same kind.  `stmt`
 * dispatches on these and `highlight` colours from them, so each group is written down once and a
 * word added to the language reaches the colouring without anybody remembering to add it twice.
 * (The kinds only one word opens — `component`, `param`, `port`, `branch` — are still a literal in
 * each place; a `match` on `&str` cannot be made exhaustive, so this is as far as the linkage
 * goes.) */
const GAUGES: [&str; 2] = ["ground", "fix"];
const ORIENTS: [&str; 2] = ["ccw", "cw"];
const BLOCKS: [&str; 3] = ["repeat", "cycle", "ring"];

/// Whether a word may stand between two links of a chain (spec §6.6).  `to` is the plain shared
/// corner; `tangent` is the drafting word, mapped per pair of kinds to the regular At-form; and
/// any binary constraint whose spec is exactly two entity slots — `perpendicular`,
/// `equal_length`, `equal_radius` — is an infix spelling of itself, the two-argument counterpart
/// of `prefix_kind` and derived from the same registry.  `close`, which seals a loop, is not a
/// joint — it stands where a link would.
fn joint_word(w: &str) -> bool {
    w == "to" || w == "tangent" || infix_kind(w).is_some()
}

/// The words that shape a statement without naming anything — a modifier the parser eats where it
/// stands.  `as` binds a name after it, which is why `highlight` treats that one specially.
const MODIFIERS: [&str; 8] =
    ["over", "as", "at", "about", "construction", "where", "bearing", "from"];

/// What the word *after* this one is expected to be — the whole of the state the colouring carries
/// from one token to the next, and four states rather than the four independent flags that would
/// spell out twelve combinations the language never reaches.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Next {
    /// begins a statement, so it is the word that says what the statement *is*
    Start,
    /// names what the statement declares
    Def,
    /// names the component an instance is of — the one type written without a `:` before it
    Inst,
    /// nothing in particular
    Word,
}

/// Colour a program.
///
/// Spans, in order, over the classified runs only: whatever falls between two of them is ordinary
/// text and a caller writes it plainly, so nothing here has to describe whitespace.  Never fails —
/// a program half-typed is exactly the one being looked at, and it is coloured as far as it goes.
pub fn highlight(src: &str) -> Vec<(Tint, Span)> {
    if src.len() > MAX_TEXT {
        return Vec::new();
    }
    let (lexed, _) = lex(src);
    let mut out: Vec<(Tint, Span)> = Vec::with_capacity(lexed.toks.len());
    let mut at = Next::Start;
    for (i, (t, span)) in lexed.toks.iter().enumerate() {
        let prev = i.checked_sub(1).map(|j| &lexed.toks[j].0);
        let next = lexed.toks.get(i + 1).map(|(t, _)| t);
        let tint = match t {
            Tok::Nl => {
                at = Next::Start;
                continue;
            }
            Tok::Str(_) => Some(Tint::Str),
            Tok::Num(_) => Some(Tint::Num),
            Tok::Eq => Some(Tint::Seed),
            Tok::EqEq => Some(Tint::Claim),
            // a body is made of statements, so a brace begins one the way a newline does
            Tok::P('{') | Tok::P('}') => {
                at = Next::Start;
                None
            }
            Tok::P(_) => None,
            Tok::Ident(w) => {
                let (tint, then) = tint_word(w, prev, next, at);
                at = then;
                tint
            }
        };
        // anything at all leaves the opening word behind; a name the statement is still owed
        // (`Def`, `Inst`) survives the punctuation in between, which is why only `Start` lapses
        if at == Next::Start && !matches!(t, Tok::P('{') | Tok::P('}')) {
            at = Next::Word;
        }
        if let Some(tint) = tint {
            out.push((tint, *span));
        }
    }
    if lexed.comments.is_empty() {
        return out;
    }
    // the comments were never tokens; put them back where they were written.  Both runs are
    // already in order — the tokens by the walk above and the comments by the lexer — so this is
    // a merge, and sorting the two together would be throwing that away and buying it back.
    let mut merged = Vec::with_capacity(out.len() + lexed.comments.len());
    let mut cs = lexed.comments.into_iter().peekable();
    for run in out {
        while cs.peek().is_some_and(|c| c.lo < run.1.lo) {
            merged.push((Tint::Comment, cs.next().expect("just peeked")));
        }
        merged.push(run);
    }
    merged.extend(cs.map(|c| (Tint::Comment, c)));
    merged
}

/// What one word is, given where in its statement it fell, and what the word after it will be.
/// Split out because it is the whole of the rule and the loop around it is only bookkeeping.
fn tint_word(w: &str, prev: Option<&Tok>, next: Option<&Tok>, at: Next) -> (Option<Tint>, Next) {
    match at {
        Next::Def => (Some(Tint::Def), Next::Word),
        Next::Inst => (Some(Tint::Type), Next::Word),
        Next::Start => {
            // `point p`, `component Gear(…)`, `curve involute(…)`, `param R = …`, `port lo: point`
            if EntKind::parse(w).is_some() || matches!(w, "component" | "param" | "port") {
                return (Some(Tint::Word), Next::Def);
            }
            if BLOCKS.contains(&w) {
                return (Some(Tint::Word), Next::Word);
            }
            // the gauges and the orientations: statements the parser knows by name, which are
            // relations in everything but where they are written down
            if GAUGES.contains(&w) || ORIENTS.contains(&w) || w == "branch" {
                return (Some(Tint::Relation), Next::Word);
            }
            // `t: Tooth(…)` — a name, a colon and a component
            if next == Some(&Tok::P(':')) {
                return (Some(Tint::Def), Next::Inst);
            }
            // `trace p where { … }` — a family body usually starts its own line
            if w == "trace" {
                return (Some(Tint::Word), Next::Def);
            }
            (CKind::from_name(&camel(w)).is_some().then_some(Tint::Relation), Next::Word)
        }
        Next::Word => {
            // `c: circle`, `phase: Angle` — the one place a bare word is a type
            if prev == Some(&Tok::P(':')) && Ty::parse(w).is_some() {
                return (Some(Tint::Type), Next::Word);
            }
            if next == Some(&Tok::P(':')) {
                return (Some(Tint::Label), Next::Word);
            }
            if MODIFIERS.contains(&w) {
                // `cycle N as i` — the binder is a name the block declares
                return (Some(Tint::Word), if w == "as" { Next::Def } else { Next::Word });
            }
            // `= trace p where { … }` — the traced point is a name the family declares
            if w == "trace" && prev == Some(&Tok::Eq) {
                return (Some(Tint::Word), Next::Def);
            }
            // a chain (spec §6.6): the element keyword mid-line, the words standing prefix to
            // it, the joints between links, and `close`.  Each is claimed only in the company a
            // chain puts it in, so a point *named* `tangent` in an argument list stays plain.
            let next_word = match next {
                Some(Tok::Ident(n)) => Some(n.as_str()),
                _ => None,
            };
            let at_line_end = matches!(next, Some(Tok::Nl) | None);
            if opens_link(w, next_word) {
                // the element keyword names what the link declares; a prefix states a relation
                return match EntKind::parse(w) {
                    Some(_) => (Some(Tint::Word), Next::Def),
                    None => (Some(Tint::Relation), Next::Word),
                };
            }
            if (next_word.is_some() || at_line_end) && joint_word(w) {
                return (Some(if w == "to" { Tint::Word } else { Tint::Relation }), Next::Word);
            }
            if w == "close" && at_line_end {
                return (Some(Tint::Word), Next::Word);
            }
            (None, Next::Word)
        }
    }
}

/* -- chains ------------------------------------------------------------------------ */

/// One link of a chain while it is being read: the unary constraint words standing before it,
/// the declaration itself, and where its text sits.
struct Link {
    prefixes: Vec<(CKind, Span)>,
    decl: Decl,
    /// Where the link's text runs, which is not the declaration's: it starts at the element
    /// keyword rather than at the name.
    span: Span,
}

/// The constraint a chain word names, and how many entities it relates — the one lookup behind
/// both `prefix_kind` and `infix_kind`, which are asked of the same word in turn.
///
/// Registry-derived, so a future unary or binary constraint joins the chain grammar without
/// anybody remembering to list it here.  It allocates (`camel`) and scans the registry, so every
/// caller guards it with the cheap question first — the colouring asks it per identifier per
/// keystroke.
fn chain_kind(w: &str) -> Option<(CKind, usize)> {
    let k = CKind::from_name(&camel(w))?;
    let spec = k.spec();
    spec.iter().all(|(_, s)| s.is_entity()).then_some((k, spec.len()))
}

/// The unary constraint a word names — `horizontal`, `vertical` — eligible to stand prefix to a
/// declaration.
fn prefix_kind(w: &str) -> Option<CKind> {
    chain_kind(w).filter(|&(_, n)| n == 1).map(|(k, _)| k)
}

/// The binary constraint a word names infix: `perpendicular`, `equal_length`, `equal_radius`.
fn infix_kind(w: &str) -> Option<CKind> {
    chain_kind(w).filter(|&(_, n)| n == 2).map(|(k, _)| k)
}

/// Whether `w`, with `next` the identifier after it, opens a chain link — an element keyword
/// naming what it declares, or a prefix word standing before one.
///
/// **The one reading**, asked by the parser (`P::chain_starts`) and by the colouring
/// (`tint_word`) alike.  A prefix word qualifies an element only when an element — or another
/// prefix — follows it, which is what keeps `horizontal(bottom)` the longhand statement it has
/// always been; written twice, the two copies drifted on exactly that clause, and a colour that
/// disagrees with the parser is the one thing `highlight` exists to rule out.
fn opens_link(w: &str, next: Option<&str>) -> bool {
    // the lookahead first: it is a pointer test, where `prefix_kind` allocates
    let Some(n) = next else { return false };
    if EntKind::parse(w).is_some() {
        return true; // a declaration names itself
    }
    (EntKind::parse(n).is_some() || prefix_kind(n).is_some()) && prefix_kind(w).is_some()
}


/// What the field at a boundary slot is called, for a message about it.
fn boundary_name(k: EntKind, slot: usize) -> &'static str {
    k.fields()
        .iter()
        .filter(|(_, f)| *f != Field::Scalar)
        .nth(slot)
        .map(|(n, _)| *n)
        .unwrap_or("end")
}

/// Whether two references name the same thing — the comparison the two sides of a joint are
/// held to.  Not `==`: a `Ref` carries the span it was written at, so the same name written in
/// two places is two unequal values.
fn refs_eq(a: &Ref, b: &Ref) -> bool {
    a.root.text == b.root.text
        && a.path.len() == b.path.len()
        && a.path.iter().zip(&b.path).all(|(x, y)| match (x, y) {
            (Seg::Field(f), Seg::Field(g)) => f.text == g.text,
            (Seg::Index(i), Seg::Index(j)) => i == j,
            _ => false,
        })
}

/// A reference as written, for a message about it.
fn ref_text(r: &Ref) -> String {
    let mut s = String::new();
    write_ref(&mut s, r);
    s
}

/* -- parsing ----------------------------------------------------------------------- */

struct P<'a> {
    src: &'a str,
    t: Vec<(Tok, Span)>,
    i: usize,
    errs: Vec<SynErr>,
}

/// Read a program.
///
/// Never fails: a statement that cannot be read is reported and skipped, and the parser resyncs
/// at the next statement terminator — so one bad line costs one line and the rest of the drawing
/// still arrives.  That is the same bargain `program::elaborate` strikes, for the same reason: a
/// panel has to show the drawing *and* the error.
pub fn parse(src: &str) -> (Program, Vec<SynErr>) {
    let mut p = Program::new();
    p.text = src.to_string();
    if src.len() > MAX_TEXT {
        return (
            p,
            vec![SynErr {
                span: Span::new(0, 0),
                message: format!("a program may not be longer than {MAX_TEXT} bytes"),
            }],
        );
    }
    let (lexed, errs) = lex(src);
    let mut st = P { src, t: lexed.toks, i: 0, errs };
    let mut body: Vec<Stmt> = Vec::new();
    let mut comps: Vec<Component> = Vec::new();
    let mut families: Vec<CurveFamily> = Vec::new();
    let mut next_id = 0u32;
    while !st.done() {
        st.skip_ends();
        if st.done() {
            break;
        }
        if body.len() >= MAX_STMTS {
            st.errs.push(SynErr {
                span: st.here(),
                message: format!("a program may not hold more than {MAX_STMTS} statements"),
            });
            break;
        }
        if st.peek_word("component") {
            match st.component(&mut next_id) {
                Some(c) => comps.push(c),
                None => st.resync(),
            }
            continue;
        }
        // `curve name(` defines a *family*; `curve name =` draws one.  Which it is is settled by
        // the token after the name, and nowhere else.
        if st.peek_word("curve") && st.curve_is_family() {
            match st.curve_family(&mut next_id) {
                Some(c) => families.push(c),
                None => st.resync(),
            }
            continue;
        }
        if st.chain_or_one(&mut next_id, &mut body).is_none() {
            st.resync();
        }
    }
    p.next_stmt = next_id;
    p.curves = families;
    // named components first, the anonymous root last — `Program::root` takes the last, and a
    // program that declares components and nothing loose has its last component as the root
    let anon_empty = body.is_empty();
    p.components = comps;
    if !anon_empty || p.components.is_empty() {
        p.components.push(Component {
            name: None,
            formals: Vec::new(),
            body,
            span: Span::new(0, src.len()),
        });
    }
    let errs = std::mem::take(&mut st.errs);
    (p, errs)
}

impl<'a> P<'a> {
    fn done(&self) -> bool {
        self.i >= self.t.len()
    }

    fn peek(&self) -> Option<&Tok> {
        self.t.get(self.i).map(|(t, _)| t)
    }

    fn here(&self) -> Span {
        self.t.get(self.i).map(|(_, s)| *s).unwrap_or(Span::new(self.src.len(), self.src.len()))
    }

    /// The source from an offset to wherever the last token ended.
    ///
    /// Clamped, because a run that consumed no tokens leaves the end *before* the start, and a
    /// reversed range is a panic — which on `wasm32-unknown-unknown` is an abort, so a program
    /// that is not a program would take the page with it.
    fn text_from(&self, from: usize) -> &'a str {
        let lo = from.min(self.src.len());
        let hi = self.prev_hi().clamp(lo, self.src.len());
        self.src.get(lo..hi).unwrap_or("")
    }

    fn prev_hi(&self) -> usize {
        self.t.get(self.i.saturating_sub(1)).map(|(_, s)| s.hi as usize).unwrap_or(self.src.len())
    }

    fn bump(&mut self) -> Option<(Tok, Span)> {
        let v = self.t.get(self.i).cloned();
        if v.is_some() {
            self.i += 1;
        }
        v
    }

    fn eat_p(&mut self, c: char) -> bool {
        if self.peek() == Some(&Tok::P(c)) {
            self.i += 1;
            true
        } else {
            false
        }
    }

    fn want_p(&mut self, c: char) -> bool {
        if self.eat_p(c) {
            return true;
        }
        self.fail(&format!("expected `{c}`"));
        false
    }

    fn eat_word(&mut self, w: &str) -> bool {
        if matches!(self.peek(), Some(Tok::Ident(s)) if s == w) {
            self.i += 1;
            true
        } else {
            false
        }
    }

    fn ident(&mut self) -> Option<Name> {
        match self.bump() {
            Some((Tok::Ident(s), span)) => Some(Name { text: s, span }),
            _ => {
                self.i = self.i.saturating_sub(1);
                self.fail("expected a name");
                None
            }
        }
    }

    fn fail(&mut self, msg: &str) {
        let span = self.here();
        // one message per position: a cascade after the first is noise
        if self.errs.last().map(|e| e.span.lo) != Some(span.lo) {
            self.errs.push(SynErr { span, message: msg.to_string() });
        }
    }

    fn skip_ends(&mut self) {
        while self.peek() == Some(&Tok::Nl) {
            self.i += 1;
        }
    }

    /// Past the next statement terminator, so one bad statement costs one statement.
    fn resync(&mut self) {
        while !self.done() && self.peek() != Some(&Tok::Nl) {
            self.i += 1;
        }
        self.skip_ends();
    }

    fn number(&mut self) -> Option<f64> {
        let neg = if self.eat_p('-') {
            true
        } else {
            self.eat_p('+');
            false
        };
        match self.bump() {
            Some((Tok::Num(v), _)) => Some(if neg { -v } else { v }),
            _ => {
                self.i = self.i.saturating_sub(1);
                self.fail("expected a number");
                None
            }
        }
    }

    /// A value written where a number goes.  Plain digits come back as the number they are;
    /// anything else — `Rr`, `R + m`, `tau / N` — comes back as text for expansion to work out
    /// against the parameters in scope.
    fn value_text(&mut self) -> Option<(Option<f64>, String, Span)> {
        let from = self.here().lo as usize;
        let mut depth = 0i32;
        while !self.done() {
            match self.peek() {
                Some(Tok::P('(')) | Some(Tok::P('[')) => depth += 1,
                Some(Tok::P(')')) | Some(Tok::P(']')) if depth == 0 => break,
                Some(Tok::P(')')) | Some(Tok::P(']')) => depth -= 1,
                Some(Tok::P(',')) if depth == 0 => break,
                Some(Tok::Nl) => break,
                _ => {}
            }
            self.i += 1;
        }
        let text = self.text_from(from).trim().to_string();
        if text.is_empty() {
            self.fail("expected a number");
            return None;
        }
        let span = Span::new(from, self.prev_hi());
        Some((text.parse::<f64>().ok().filter(|v| v.is_finite()), text, span))
    }

    fn refr(&mut self) -> Option<Ref> {
        let root = self.ident()?;
        let lo = root.span.lo as usize;
        let mut path = Vec::new();
        loop {
            if self.eat_p('.') {
                path.push(Seg::Field(self.ident()?));
            } else if self.eat_p('[') {
                let (text, _) = self.expr_until(']')?;
                if !self.want_p(']') {
                    return None;
                }
                path.push(Seg::Index(text));
            } else {
                break;
            }
        }
        Some(Ref { root, path, span: Span::new(lo, self.prev_hi()) })
    }

    fn stmt(&mut self, next_id: &mut u32) -> Option<StmtKind> {
        let Some(Tok::Ident(w)) = self.peek().cloned() else {
            self.fail("a statement starts with a word");
            return None;
        };
        match w.as_str() {
            g if GAUGES.contains(&g) => {
                self.i += 1;
                let ground = w == "ground";
                if !self.want_p('(') {
                    return None;
                }
                let r = self.refr()?;
                if !self.want_p(')') {
                    return None;
                }
                Some(StmtKind::Gauge(if ground { Gauge::Ground(r) } else { Gauge::Fix(r) }))
            }
            o if ORIENTS.contains(&o) => {
                self.i += 1;
                let ccw = w == "ccw";
                if !self.want_p('(') {
                    return None;
                }
                let mut pts = Vec::new();
                while !self.eat_p(')') {
                    pts.push(self.refr()?);
                    if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                        self.fail("expected `,` or `)`");
                        return None;
                    }
                }
                Some(StmtKind::Orient(Orient { ccw, pts, raw: None }))
            }
            "branch" => {
                self.i += 1;
                if !self.want_p('(') {
                    return None;
                }
                let key = match self.bump() {
                    Some((Tok::Str(s), _)) => s,
                    _ => {
                        self.fail("a raw branch key is a string");
                        return None;
                    }
                };
                if !self.want_p(',') {
                    return None;
                }
                let v = self.number()? as i32;
                if !self.want_p(')') {
                    return None;
                }
                Some(StmtKind::Orient(Orient { ccw: v >= 0, pts: Vec::new(), raw: Some((key, v)) }))
            }
            "port" => {
                self.i += 1;
                let name = self.ident()?;
                if self.eat_p(':') {
                    let ty = self.ident()?;
                    let Some(k) = EntKind::parse(&ty.text.to_lowercase()) else {
                        self.errs.push(SynErr {
                            span: ty.span,
                            message: format!("`{}` is not a kind of entity", ty.text),
                        });
                        return None;
                    };
                    self.end_of_stmt();
                    Some(StmtKind::Port(Port { name, declare: Some(k), alias: None }))
                } else if self.peek() == Some(&Tok::Eq) {
                    self.i += 1;
                    let r = self.refr()?;
                    self.end_of_stmt();
                    Some(StmtKind::Port(Port { name, declare: None, alias: Some(r) }))
                } else {
                    self.fail("a port is `port name: Kind` or `port name = other`");
                    None
                }
            }
            "param" => {
                self.i += 1;
                let name = self.ident()?;
                if self.peek() != Some(&Tok::Eq) {
                    self.fail("a param is `param name = expression`");
                    return None;
                }
                let after = self.here().hi as usize;
                self.i += 1;
                let (text, span, _, end) = self.raw_dimension(after);
                while self.i < self.t.len() && (self.t[self.i].1.lo as usize) < end {
                    self.i += 1;
                }
                Some(StmtKind::Param(ParamDecl { name, text, span }))
            }
            b if BLOCKS.contains(&b) => {
                self.i += 1;
                let kind = match w.as_str() {
                    "repeat" => BlockKind::Repeat,
                    "cycle" => BlockKind::Cycle,
                    _ => BlockKind::Ring,
                };
                self.block(kind, next_id).map(StmtKind::Block)
            }
            // `t: Tooth(...)` — a name, a colon and a component
            _ if matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P(':'))) => {
                let name = self.ident()?;
                self.i += 1; // the colon
                let component = self.ident()?;
                let lo = name.span.lo as usize;
                let mut args = Vec::new();
                if self.want_p('(') {
                    while !self.eat_p(')') {
                        args.push(self.inst_arg()?);
                        if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                            self.fail("expected `,` or `)`");
                            return None;
                        }
                    }
                } else {
                    return None;
                }
                self.end_of_stmt();
                Some(StmtKind::Instance(Instance {
                    name,
                    component,
                    args,
                    span: Span::new(lo, self.prev_hi()),
                }))
            }
            _ => self.relation().map(StmtKind::Relation),
        }
    }

    /// One statement — or a chain, desugared here into the statements it is sugar for.
    ///
    /// A chain is a *parser addition rather than a change of shape*, the bargain `component`
    /// struck: what comes out is ordinary declarations and relations, each with its own id and a
    /// span into the chain's own text, so nothing downstream — `flatten`, the elaborator, the
    /// source map, a splice — learns the word "chain".  Spec §6.6.
    fn chain_or_one(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        if !self.chain_starts() {
            let lo = self.here().lo as usize;
            let kind = self.stmt(next_id)?;
            *next_id += 1;
            out.push(Stmt {
                id: StmtId(*next_id),
                kind,
                span: Span::new(lo, self.prev_hi()),
                chained: Chained::No,
            });
            return Some(());
        }
        self.chain(next_id, out)
    }

    /// Whether what stands here opens a declaration — possibly a chain of them.
    fn chain_starts(&self) -> bool {
        let Some(Tok::Ident(w)) = self.peek() else { return false };
        opens_link(w, self.word_at(self.i + 1))
    }

    /// The identifier at a token position, where there is one — what `opens_link` reads ahead.
    fn word_at(&self, i: usize) -> Option<&str> {
        match self.t.get(i).map(|(t, _)| t) {
            Some(Tok::Ident(n)) => Some(n.as_str()),
            _ => None,
        }
    }

    /// `[prefix…] decl (joint [prefix…] decl)* [joint "close"]`.
    fn chain(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        let mut links = vec![self.link()?];
        let mut joints: Vec<(String, Span)> = Vec::new();
        let mut close: Option<(String, Span)> = None;
        loop {
            let joint = match self.peek() {
                Some(Tok::Ident(w)) if joint_word(w) => w.clone(),
                _ => break,
            };
            let wspan = self.here();
            self.i += 1;
            // a line ending in a joint word continues the chain on the next — the one place a
            // statement runs past its line's end
            self.skip_ends();
            if self.eat_word("close") {
                close = Some((joint, Span::new(wspan.lo as usize, self.prev_hi())));
                break;
            }
            joints.push((joint, wspan));
            links.push(self.link()?);
        }
        self.end_of_stmt();
        self.desugar(links, joints, close, next_id, out);
        Some(())
    }

    /// `[prefix…] KIND name(…)` — the declaration a chain link is.
    fn link(&mut self) -> Option<Link> {
        let mut prefixes: Vec<(CKind, Span)> = Vec::new();
        let kind = loop {
            let Some(Tok::Ident(w)) = self.peek() else {
                self.fail("expected an element");
                return None;
            };
            if let Some(k) = EntKind::parse(w) {
                break k;
            }
            match prefix_kind(w) {
                Some(c) => {
                    prefixes.push((c, self.here()));
                    self.i += 1;
                }
                None => {
                    self.fail("expected an element");
                    return None;
                }
            }
        };
        let lo = self.here().lo as usize;
        self.i += 1; // the kind keyword
        let decl = self.decl(kind)?;
        Some(Link { prefixes, decl, span: Span::new(lo, self.prev_hi()) })
    }

    /// The statements a chain is sugar for, in the order their text sits: each link's prefix
    /// relations, its declaration, and between links the joint that binds them.
    fn desugar(
        &mut self,
        mut links: Vec<Link>,
        joints: Vec<(String, Span)>,
        close: Option<(String, Span)>,
        next_id: &mut u32,
        out: &mut Vec<Stmt>,
    ) {
        let chained = links.len() > 1 || close.is_some();
        let n = links.len();
        // a chain threads through boundary points, so every link must have a boundary — which is
        // exactly a line or an arc.  A lone (possibly prefixed) declaration may be any kind.
        let mut sound = true;
        if chained {
            for l in &links {
                if l.decl.kind.ends().is_none() {
                    self.errs.push(SynErr {
                        span: l.decl.name.span,
                        message: format!(
                            "a chain joins lines and arcs; a {} has no ends to thread",
                            l.decl.kind.as_str()
                        ),
                    });
                    sound = false;
                }
            }
            if close.is_some() && n < 2 {
                self.errs.push(SynErr {
                    span: links[0].decl.name.span,
                    message: "a chain closes over at least two elements".to_string(),
                });
                sound = false;
            }
        }
        // threading: at each joint the shared point is named by exactly one side, or by both in
        // agreement, and the name fills whichever side left its boundary field out
        if chained && sound {
            for i in 0..n - 1 {
                self.thread(&mut links, i, i + 1, joints[i].1);
            }
            match &close {
                Some((_, sp)) => self.thread(&mut links, n - 1, 0, *sp),
                None => {
                    self.loose_end(&links[0], true);
                    self.loose_end(&links[n - 1], false);
                }
            }
        }
        // the links are consumed below, so what a joint needs of them — a name and a kind — is
        // taken first, and only where there are joints to need it
        let sig: Vec<(String, EntKind)> = match chained {
            true => links.iter().map(|l| (l.decl.name.text.clone(), l.decl.kind)).collect(),
            false => Vec::new(),
        };
        let at = |i: usize| (sig[i].0.as_str(), sig[i].1);
        let first = out.len();
        for (i, link) in links.into_iter().enumerate() {
            if i > 0 && sound {
                let (w, sp) = &joints[i - 1];
                out.extend(self.joint_stmt(w, *sp, at(i - 1), at(i), Chained::Joint, next_id));
            }
            for (k, sp) in &link.prefixes {
                let rel = Relation {
                    kind: *k,
                    args: vec![Some(Arg::Ref(Ref {
                        root: Name { text: link.decl.name.text.clone(), span: *sp },
                        path: Vec::new(),
                        span: *sp,
                    }))],
                    place: None,
                };
                *next_id += 1;
                out.push(Stmt {
                    id: StmtId(*next_id),
                    kind: StmtKind::Relation(rel),
                    span: *sp,
                    chained: Chained::Prefix,
                });
            }
            *next_id += 1;
            out.push(Stmt {
                id: StmtId(*next_id),
                kind: StmtKind::Decl(link.decl),
                span: link.span,
                chained: if chained { Chained::Link } else { Chained::No },
            });
        }
        if let Some((w, sp)) = close {
            let sealed =
                sound.then(|| self.joint_stmt(&w, sp, at(n - 1), at(0), Chained::Close, next_id));
            match sealed.flatten() {
                Some(st) => out.push(st),
                // `to close` states nothing, so no statement owns its words; the last link's
                // span grows over them, or an append would land in the middle of the chain
                None => {
                    if let Some(last) = out.last_mut() {
                        last.span = Span::new(last.span.lo as usize, sp.hi as usize);
                    }
                }
            }
        }
        debug_assert!(out.len() > first, "a chain states at least its own declaration");
    }

    /// The statement one joint states, where it states one — a plain corner states nothing.
    fn joint_stmt(
        &mut self,
        word: &str,
        at: Span,
        left: (&str, EntKind),
        right: (&str, EntKind),
        chained: Chained,
        next_id: &mut u32,
    ) -> Option<Stmt> {
        let rel = self.joint_relation(word, at, left, right)?;
        *next_id += 1;
        Some(Stmt { id: StmtId(*next_id), kind: StmtKind::Relation(rel), span: at, chained })
    }

    /// Resolve one joint's shared point between link `li` (its exit) and link `ri` (its entry).
    fn thread(&mut self, links: &mut [Link], li: usize, ri: usize, at: Span) {
        let (Some((_, exit)), Some((entry, _))) =
            (links[li].decl.kind.ends(), links[ri].decl.kind.ends())
        else {
            return; // a kind with no ends, already reported
        };
        let left = links[li].decl.children.get(exit).and_then(|v| v.first()).cloned();
        let right = links[ri].decl.children.get(entry).and_then(|v| v.first()).cloned();
        match (left, right) {
            (Some(l), Some(r)) => {
                if !refs_eq(&l, &r) {
                    self.errs.push(SynErr {
                        span: at,
                        message: format!(
                            "the joint names two points: `{}` leaves at `{}` and `{}` arrives \
                             at `{}`",
                            links[li].decl.name.text,
                            ref_text(&l),
                            links[ri].decl.name.text,
                            ref_text(&r),
                        ),
                    });
                }
            }
            (Some(l), None) => links[ri].decl.children[entry] = vec![l],
            (None, Some(r)) => links[li].decl.children[exit] = vec![r],
            (None, None) => {
                self.errs.push(SynErr {
                    span: at,
                    message: format!(
                        "neither `{}` nor `{}` names the point where they meet",
                        links[li].decl.name.text, links[ri].decl.name.text
                    ),
                });
            }
        }
    }

    /// An open chain's first entry and last exit are not joints, so nothing fills them in; they
    /// must be named where they stand.
    fn loose_end(&mut self, l: &Link, entry: bool) {
        let Some((en, ex)) = l.decl.kind.ends() else { return };
        let slot = if entry { en } else { ex };
        if l.decl.children.get(slot).is_none_or(|v| v.is_empty()) {
            let field = boundary_name(l.decl.kind, slot);
            self.errs.push(SynErr {
                span: l.decl.name.span,
                message: format!("the chain leaves `{}`'s {field} unnamed", l.decl.name.text),
            });
        }
    }

    /// The relation one qualified joint states, or `None` for a plain corner — and an error
    /// where the vocabulary has no regular form for the pair, which is refused rather than
    /// stated as a bare tangency over a shared point (a double root no rank tolerance can read).
    fn joint_relation(
        &mut self,
        word: &str,
        at: Span,
        left: (&str, EntKind),
        right: (&str, EntKind),
    ) -> Option<Relation> {
        use EntKind::{Arc, Line};
        let ent = |name: &str| {
            Some(Arg::Ref(Ref {
                root: Name { text: name.to_string(), span: at },
                path: Vec::new(),
                span: at,
            }))
        };
        let (lname, lk) = left;
        let (rname, rk) = right;
        match (word, lk, rk) {
            ("to", _, _) => None,
            // the joint knows the shared point, so tangency is stated *at* it — the regular
            // form, with `at:` read off the direction of travel
            ("tangent", Line, Arc) => Some(Relation {
                kind: CKind::TangentArcLine,
                args: vec![ent(rname), ent(lname), Some(Arg::Word("start".to_string()))],
                place: None,
            }),
            ("tangent", Arc, Line) => Some(Relation {
                kind: CKind::TangentArcLine,
                args: vec![ent(lname), ent(rname), Some(Arg::Word("end".to_string()))],
                place: None,
            }),
            // two straight runs meeting tangent share a point and a direction: collinear
            ("tangent", Line, Line) => Some(Relation {
                kind: CKind::Parallel,
                args: vec![ent(lname), ent(rname)],
                place: None,
            }),
            _ => {
                // any binary constraint whose slots the pair fits is an infix spelling of
                // itself: `perpendicular`, `equal_length`, `equal_radius`, …
                if let Some(k) = infix_kind(word) {
                    let spec = k.spec();
                    if crate::constraints::kind_matches(spec[0].1, lk)
                        && crate::constraints::kind_matches(spec[1].1, rk)
                    {
                        return Some(Relation {
                            kind: k,
                            args: vec![ent(lname), ent(rname)],
                            place: None,
                        });
                    }
                }
                self.errs.push(SynErr {
                    span: at,
                    message: format!(
                        "`{word}` does not join a {} to a {}",
                        lk.as_str(),
                        rk.as_str()
                    ),
                });
                None
            }
        }
    }

    /// After `curve NAME`, a `(` opens a family's formals and anything else is an instance.
    fn curve_is_family(&self) -> bool {
        matches!(self.t.get(self.i + 2).map(|(t, _)| t), Some(Tok::P('(')))
    }

    /// `curve NAME(formals)(param) [over (a, b)] = ( xexpr, yexpr )`, or
    /// `… = trace NAME where { … }`.
    fn curve_family(&mut self, next_id: &mut u32) -> Option<CurveFamily> {
        let lo = self.here().lo as usize;
        self.i += 1; // `curve`
        let name = self.ident()?;
        let mut formals = Vec::new();
        if self.want_p('(') {
            while !self.eat_p(')') {
                let fname = self.ident()?;
                if !self.want_p(':') {
                    return None;
                }
                let tname = self.ident()?;
                let Some(ty) = Ty::parse(&tname.text) else {
                    self.errs.push(SynErr {
                        span: tname.span,
                        message: format!("`{}` is not a type", tname.text),
                    });
                    return None;
                };
                let span = Span::new(fname.span.lo as usize, self.prev_hi());
                formals.push(Formal { name: fname, ty, span });
                if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                    self.fail("expected `,` or `)`");
                    return None;
                }
            }
        } else {
            return None;
        }
        // the parameter it runs on
        if !self.want_p('(') {
            return None;
        }
        let param = self.ident()?;
        if !self.want_p(')') {
            return None;
        }
        let domain = if self.eat_word("over") { Some(self.interval()?) } else { None };
        if self.peek() != Some(&Tok::Eq) {
            self.fail("a curve family is `curve name(...)(u) = ( x, y )`");
            return None;
        }
        self.i += 1;
        // a family is usually too long for one line, and the `=` is where it breaks
        self.skip_ends();
        // `trace p [from (expr)] where { … }` — the locus form
        if self.eat_word("trace") {
            let point = self.ident()?;
            let home =
                if self.eat_word("from") { Some(self.paren_expr()?) } else { None };
            if !self.eat_word("where") {
                self.fail("a trace is `trace point [from (...)] where { ... }`");
                return None;
            }
            let body = self.braced_body(next_id)?;
            return Some(CurveFamily {
                name,
                formals,
                param,
                domain,
                body: FamilyBody::Trace { point, home, body },
                span: Span::new(lo, self.prev_hi()),
            });
        }
        if !self.want_p('(') {
            return None;
        }
        let (x, xspan) = self.expr_until(',')?;
        if !self.want_p(',') {
            return None;
        }
        let (y, yspan) = self.expr_until(')')?;
        if !self.want_p(')') {
            return None;
        }
        self.end_of_stmt();
        Some(CurveFamily {
            name,
            formals,
            param,
            domain,
            body: FamilyBody::Exprs { x, y, xspan, yspan },
            span: Span::new(lo, self.prev_hi()),
        })
    }

    /// `( expr )` — the parenthesised expression `from` and `bearing` both carry.
    fn paren_expr(&mut self) -> Option<(String, Span)> {
        if !self.want_p('(') {
            return None;
        }
        let e = self.expr_until(')')?;
        if !self.want_p(')') {
            return None;
        }
        Some(e)
    }

    /// `over (a, b)` — two expressions over whatever parameters are in scope.
    fn interval(&mut self) -> Option<(String, String)> {
        if !self.want_p('(') {
            return None;
        }
        let (a, _) = self.expr_until(',')?;
        if !self.want_p(',') {
            return None;
        }
        let (b, _) = self.expr_until(')')?;
        if !self.want_p(')') {
            return None;
        }
        Some((a, b))
    }

    /// The source up to `end` at the top bracket level, as written.
    fn expr_until(&mut self, end: char) -> Option<(String, Span)> {
        let from = self.here().lo as usize;
        let mut depth = 0i32;
        while !self.done() {
            match self.peek() {
                Some(Tok::P('(')) | Some(Tok::P('[')) => depth += 1,
                Some(Tok::P(c)) if *c == end && depth == 0 => break,
                Some(Tok::P(')')) | Some(Tok::P(']')) if depth == 0 => break,
                Some(Tok::P(')')) | Some(Tok::P(']')) => depth -= 1,
                Some(Tok::Nl) => break,
                _ => {}
            }
            self.i += 1;
        }
        let text = self.text_from(from).trim().to_string();
        if text.is_empty() {
            self.fail("expected an expression");
            return None;
        }
        Some((text, Span::new(from, self.prev_hi())))
    }

    fn peek_word(&self, w: &str) -> bool {
        matches!(self.peek(), Some(Tok::Ident(s)) if s == w)
    }

    /// `component Name(formals) { body }`.
    fn component(&mut self, next_id: &mut u32) -> Option<Component> {
        let lo = self.here().lo as usize;
        self.i += 1; // `component`
        let name = self.ident()?;
        let mut formals = Vec::new();
        if self.eat_p('(') {
            while !self.eat_p(')') {
                let fname = self.ident()?;
                if !self.want_p(':') {
                    return None;
                }
                let tname = self.ident()?;
                let Some(ty) = Ty::parse(&tname.text) else {
                    self.errs.push(SynErr {
                        span: tname.span,
                        message: format!("`{}` is not a type", tname.text),
                    });
                    return None;
                };
                let span = Span::new(fname.span.lo as usize, self.prev_hi());
                formals.push(Formal { name: fname, ty, span });
                if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                    self.fail("expected `,` or `)`");
                    return None;
                }
            }
        }
        let body = self.braced_body(next_id)?;
        Some(Component { name: Some(name), formals, body, span: Span::new(lo, self.prev_hi()) })
    }

    fn block(&mut self, kind: BlockKind, next_id: &mut u32) -> Option<Block> {
        let lo = self.prev_hi();
        // the count runs to `about`, `as` or `{`, and is an expression over what is in scope
        let from = self.here().lo as usize;
        let mut depth = 0i32;
        while !self.done() {
            match self.peek() {
                Some(Tok::P('(')) => depth += 1,
                Some(Tok::P(')')) => depth -= 1,
                Some(Tok::P('{')) if depth == 0 => break,
                Some(Tok::Ident(w)) if depth == 0 && (w == "about" || w == "as") => break,
                Some(Tok::Nl) => break,
                _ => {}
            }
            self.i += 1;
        }
        let count = self.text_from(from).trim().to_string();
        let about = if self.eat_word("about") { Some(self.refr()?) } else { None };
        let binder = if self.eat_word("as") { Some(self.ident()?) } else { None };
        let body = self.braced_body(next_id)?;
        Some(Block { kind, count, about, binder, body, span: Span::new(lo, self.prev_hi()) })
    }

    fn braced_body(&mut self, next_id: &mut u32) -> Option<Vec<Stmt>> {
        self.skip_ends();
        if !self.want_p('{') {
            return None;
        }
        let mut body = Vec::new();
        loop {
            self.skip_ends();
            if self.eat_p('}') {
                break;
            }
            if self.done() {
                self.fail("a body with no closing `}`");
                return None;
            }
            if self.chain_or_one(next_id, &mut body).is_none() {
                self.resync();
                if self.done() {
                    return None;
                }
            }
        }
        Some(body)
    }

    /// One argument of an instantiation: an entity by name, or a number worked out here.
    fn inst_arg(&mut self) -> Option<InstArg> {
        let lo = self.here().lo as usize;
        let label = match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t)) {
            (Some(Tok::Ident(s)), Some(Tok::P(':'))) => {
                let n = Name { text: s, span: self.here() };
                self.i += 2;
                Some(n)
            }
            _ => None,
        };
        // a bare name is an entity; anything with an operator in it is a value expression, and
        // the two are told apart by what follows the first token rather than by a type
        let bare = matches!(self.peek(), Some(Tok::Ident(_)))
            && matches!(
                self.t.get(self.i + 1).map(|(t, _)| t),
                Some(Tok::P(',')) | Some(Tok::P(')')) | Some(Tok::P('.')) | Some(Tok::P('['))
            );
        let value = if bare {
            InstVal::Ref(self.refr()?)
        } else {
            let from = self.here().lo as usize;
            let mut depth = 0i32;
            while !self.done() {
                match self.peek() {
                    Some(Tok::P('(')) | Some(Tok::P('[')) => depth += 1,
                    Some(Tok::P(')')) | Some(Tok::P(']')) if depth == 0 => break,
                    Some(Tok::P(')')) | Some(Tok::P(']')) => depth -= 1,
                    Some(Tok::P(',')) if depth == 0 => break,
                    Some(Tok::Nl) => break,
                    _ => {}
                }
                self.i += 1;
            }
            InstVal::Expr(self.text_from(from).trim().to_string())
        };
        Some(InstArg { label, value, span: Span::new(lo, self.prev_hi()) })
    }

    fn decl(&mut self, kind: EntKind) -> Option<Decl> {
        let name = self.ident()?;
        // `curve e = involute(base, phase: 0) over (0, 45)`
        let mut def = None;
        let mut values: Vec<(Name, String)> = Vec::new();
        let mut domain = None;
        if kind == EntKind::Curve {
            if self.peek() != Some(&Tok::Eq) {
                self.fail("a curve is drawn as `curve name = family(args)`");
                return None;
            }
            self.i += 1;
            def = Some(self.ident()?);
            let mut args: Vec<Vec<Ref>> = vec![Vec::new()];
            if self.want_p('(') {
                while !self.eat_p(')') {
                    // `phase: 0` is a number the family takes; a bare name is an entity it is
                    // written over
                    let label = match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t))
                    {
                        (Some(Tok::Ident(s)), Some(Tok::P(':'))) => {
                            let n = Name { text: s, span: self.here() };
                            self.i += 2;
                            Some(n)
                        }
                        _ => None,
                    };
                    match label {
                        Some(l) => {
                            let (t, _) = self.expr_until(',')?;
                            values.push((l, t));
                        }
                        None => args[0].push(self.refr()?),
                    }
                    if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                        self.fail("expected `,` or `)`");
                        return None;
                    }
                }
            } else {
                return None;
            }
            if self.eat_word("over") {
                domain = Some(self.interval()?);
            }
            let construction = self.eat_word("construction");
            return Some(Decl {
                kind,
                name,
                children: args,
                seed: Vec::new(),
                seed_text: Vec::new(),
                seed_spans: Vec::new(),
                knots: None,
                def,
                values,
                domain,
                construction,
                seed_at: None,
            });
        }
        let mut children: Vec<Vec<Ref>> = Vec::new();
        let mut seed: Vec<f64> = Vec::new();
        let fields = kind.fields();
        // one slot per Child/List field, so the printer's shape and the parser's agree
        for (_, f) in fields {
            if *f != Field::Scalar {
                children.push(Vec::new());
            }
        }
        let scalars: Vec<&str> =
            fields.iter().filter(|(_, f)| *f == Field::Scalar).map(|(n, _)| *n).collect();
        seed.resize(scalars.len(), 0.0);
        let mut seed_text: Vec<Option<String>> = vec![None; scalars.len()];
        let mut seed_spans: Vec<Span> = vec![Span::default(); scalars.len()];
        if self.eat_p('(') {
            let mut positional = 0usize;
            while !self.eat_p(')') {
                // `name:` labels a field; anything else is positional
                let label = match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t)) {
                    (Some(Tok::Ident(s)), Some(Tok::P(':'))) => {
                        self.i += 2;
                        Some(s)
                    }
                    _ => None,
                };
                match label {
                    Some(l) if scalars.contains(&l.as_str()) => {
                        let (v, t, sp) = self.value_text()?;
                        if let Some(i) = scalars.iter().position(|&s| s == l) {
                            seed[i] = v.unwrap_or(0.0);
                            seed_text[i] = (v.is_none()).then_some(t);
                            seed_spans[i] = sp;
                        }
                    }
                    _ => {
                        let r = self.refr()?;
                        let slot = match &label {
                            Some(l) => fields
                                .iter()
                                .filter(|(_, f)| *f != Field::Scalar)
                                .position(|(n, _)| n == l)
                                .unwrap_or_else(|| positional.min(children.len() - 1)),
                            None => {
                                // a List field takes every positional argument from where it starts
                                let n_named = fields
                                    .iter()
                                    .filter(|(_, f)| *f == Field::Child)
                                    .count();
                                if positional >= n_named && children.len() > n_named {
                                    n_named
                                } else {
                                    positional.min(children.len().saturating_sub(1))
                                }
                            }
                        };
                        if let Some(g) = children.get_mut(slot) {
                            g.push(r);
                        }
                        positional += 1;
                    }
                }
                if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                    self.fail("expected `,` or `)`");
                    return None;
                }
            }
        }
        // trailing clauses, in any order: `at (x, y)` or `at REF [bearing (…)]`, `knots [...]`,
        // `construction`
        let mut knots = None;
        let mut construction = false;
        let mut seed_at = None;
        loop {
            if self.eat_word("at") {
                // a place named geometrically: `at t`, `at c bearing (u + phase)`
                if self.peek() != Some(&Tok::P('(')) {
                    let what = self.refr()?;
                    let bearing =
                        if self.eat_word("bearing") { Some(self.paren_expr()?) } else { None };
                    seed_at = Some(AtRef { what, bearing });
                    continue;
                }
                self.i += 1; // `(`
                let (x, xt, xs) = self.value_text()?;
                if !self.want_p(',') {
                    return None;
                }
                let (y, yt, ys) = self.value_text()?;
                if !self.want_p(')') {
                    return None;
                }
                for (i, (v, t, sp)) in [(x, xt, xs), (y, yt, ys)].into_iter().enumerate() {
                    if i < scalars.len() {
                        seed[i] = v.unwrap_or(0.0);
                        seed_text[i] = (v.is_none()).then_some(t);
                        seed_spans[i] = sp;
                    }
                }
            } else if self.eat_word("knots") {
                if !self.want_p('[') {
                    return None;
                }
                let mut u = Vec::new();
                while !self.eat_p(']') {
                    u.push(self.number()?);
                    if !self.eat_p(',') && self.peek() != Some(&Tok::P(']')) {
                        self.fail("expected `,` or `]`");
                        return None;
                    }
                }
                knots = Some(u);
            } else if self.eat_word("construction") {
                construction = true;
            } else {
                break;
            }
        }
        Some(Decl {
            kind,
            name,
            children,
            seed,
            seed_text,
            seed_spans,
            knots,
            def,
            values,
            domain,
            construction,
            seed_at,
        })
    }

    fn relation(&mut self) -> Option<Relation> {
        let name = self.ident()?;
        let Some(kind) = CKind::from_name(&camel(&name.text)) else {
            self.errs.push(SynErr {
                span: name.span,
                message: format!("`{}` is not a constraint", name.text),
            });
            return None;
        };
        let spec = kind.spec();
        let mut args: Vec<Option<Arg>> = vec![None; spec.len()];
        if !self.want_p('(') {
            return None;
        }
        let mut positional = 0usize;
        while !self.eat_p(')') {
            let label = match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t)) {
                (Some(Tok::Ident(s)), Some(Tok::P(':'))) => {
                    self.i += 2;
                    Some(s)
                }
                // `t = 0.37` seeds and `t == 0.37` pins: the label is the slot's own name
                (Some(Tok::Ident(s)), Some(Tok::Eq)) | (Some(Tok::Ident(s)), Some(Tok::EqEq)) => {
                    Some(s)
                }
                _ => None,
            };
            let slot = match &label {
                Some(l) => match spec.iter().position(|(n, _)| n == l) {
                    Some(i) => i,
                    None => {
                        self.fail(&format!("`{}` has no argument `{l}`", name.text));
                        return None;
                    }
                },
                None => {
                    // positional arguments fill the slots that were not labelled, in order
                    while positional < spec.len() && args[positional].is_some() {
                        positional += 1;
                    }
                    positional
                }
            };
            if slot >= spec.len() {
                self.fail(&format!("`{}` takes {} arguments", name.text, spec.len()));
                return None;
            }
            let a = self.arg(spec[slot].1)?;
            args[slot] = Some(a);
            if label.is_none() {
                positional += 1;
            }
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        // the trailing `== …`: everything to the end of the logical line, verbatim
        let mut place = None;
        if self.peek() == Some(&Tok::EqEq) {
            let after = self.here().hi as usize;
            self.i += 1;
            let (text, span, pl, end) = self.raw_dimension(after);
            place = pl;
            let tail = spec.len().saturating_sub(1);
            if spec.last().is_some_and(|(_, k)| k.is_dimension()) {
                args[tail] = Some(Arg::Dim { text, span });
            } else {
                self.errs.push(SynErr {
                    span,
                    message: format!("`{}` states no number", name.text),
                });
            }
            // skip every token the raw region swallowed
            while self.i < self.t.len() && (self.t[self.i].1.lo as usize) < end {
                self.i += 1;
            }
        }
        if place.is_none() && self.eat_word("at") {
            if !self.want_p('(') {
                return None;
            }
            let t = self.number()?;
            if !self.want_p(',') {
                return None;
            }
            let r = self.number()?;
            if !self.want_p(')') {
                return None;
            }
            place = Some((t, r));
        }
        self.end_of_stmt();
        Some(Relation { kind, args, place })
    }

    /// Everything after `==` to the end of the logical line, as written.
    ///
    /// Not tokenized: the dimension sub-language is `expr.rs`'s, and lexing it a second time here
    /// would be a second copy of rules like the one that makes `3 1/8` a number and `31/2` a
    /// division.  A trailing ` at (u, v)` is a placement rather than part of the expression —
    /// unambiguous, because a call in that language is `name(` with no space before the paren.
    fn raw_dimension(&mut self, from: usize) -> (String, Span, Option<(f64, f64)>, usize) {
        let bytes = self.src.as_bytes();
        let mut end = from;
        while end < bytes.len() && bytes[end] != b'\n' && bytes[end] != b';' {
            if bytes[end] == b'/' && matches!(bytes.get(end + 1), Some(b'/') | Some(b'*')) {
                break;
            }
            end += 1;
        }
        let mut text = &self.src[from..end];
        let mut place = None;
        if let Some(at) = text.rfind(" at (") {
            let tail = &text[at + 5..];
            if let Some(close) = tail.find(')') {
                if tail[close + 1..].trim().is_empty() {
                    let nums: Vec<f64> = tail[..close]
                        .split(',')
                        .filter_map(|s| s.trim().parse::<f64>().ok())
                        .collect();
                    if nums.len() == 2 {
                        place = Some((nums[0], nums[1]));
                        text = &text[..at];
                    }
                }
            }
        }
        // the span of the *trimmed* text, not of the slice it was cut from: an edit splices
        // this, and a span that included the space after `==` would eat it and write `==140`
        let lead = text.len() - text.trim_start().len();
        let trimmed = text.trim();
        let span = Span::new(from + lead, from + lead + trimmed.len());
        (trimmed.to_string(), span, place, end)
    }

    fn arg(&mut self, kind: SpecKind) -> Option<Arg> {
        // `t = 0.37` and `t == 0.37`: a slot the constraint owns, seeded or pinned
        if kind == SpecKind::Param {
            let mut pinned = false;
            if matches!(self.peek(), Some(Tok::Ident(_))) {
                let eq = self.t.get(self.i + 1).map(|(t, _)| t.clone());
                if matches!(eq, Some(Tok::Eq) | Some(Tok::EqEq)) {
                    pinned = eq == Some(Tok::EqEq);
                    self.i += 2;
                }
            }
            let lo = self.here();
            let (v, text, sp) = self.value_text()?;
            let _ = sp;
            return Some(match v {
                Some(value) => Arg::Seed { value, pinned },
                None => Arg::SeedExpr { text, pinned, span: Span::new(lo.lo as usize, self.prev_hi()) },
            });
        }
        match self.peek().cloned() {
            Some(Tok::Num(_)) | Some(Tok::P('-')) | Some(Tok::P('+')) => {
                let v = self.number()?;
                Some(match kind {
                    SpecKind::Int => Arg::Int(v as i64),
                    _ => Arg::Num(v),
                })
            }
            Some(Tok::Str(s)) => {
                self.i += 1;
                Some(Arg::Word(s))
            }
            Some(Tok::Ident(s)) if s == "true" || s == "false" => {
                self.i += 1;
                Some(Arg::Bool(s == "true"))
            }
            // a bare word in a Str slot is the word; anywhere else it names an entity
            Some(Tok::Ident(s)) if kind == SpecKind::Str => {
                self.i += 1;
                Some(Arg::Word(s))
            }
            Some(Tok::Ident(_)) => Some(Arg::Ref(self.refr()?)),
            _ => {
                self.fail("expected an argument");
                None
            }
        }
    }

    fn end_of_stmt(&mut self) {
        if self.done() || self.peek() == Some(&Tok::Nl) {
            return;
        }
        self.fail("more on this line than the statement wanted");
    }
}
