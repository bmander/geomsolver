//! Solvent: the language a sketch is written in.
//!
//! Tokens, spans, the syntax tree and the printer.  The parser joins them here too, for the reason
//! `io.rs` holds `to_json` and `from_json` together: the round trip *is* the contract, and two
//! halves of one agreement are best read side by side.
//!
//! **Nothing here is written per constraint type.**  A statement's name is the snake_case of
//! `CKind::name()`, its arguments follow `CKind::spec()`, and a trailing `Length`/`Angle` slot
//! prints after `==` — the same bargain `report::registry_json` already strikes with the
//! TypeScript binding, so a new constraint type appears in the language with nothing to
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
//! One clause carries the whole hint/constraint classification the language rests on:
//!
//! * **A number inside a `hint(…)` clause is a seed.**  `point p hint(x: 0, y: 0)`,
//!   `circle c(center: o) hint(r: 25)`, `point_on_spline(p, s) hint(t: 0.37)` are inert —
//!   deleting every one of them changes no solution set, only which solution is found, so a
//!   solve may write them back.  The brackets after a name are what the thing is *made of*;
//!   the clause after them is where the solve *begins*.
//! * **Every other number is not.**  `== 80` and `t == 0.37` state something and a solve must
//!   never rewrite one; `param w = 100` is arithmetic done while elaborating.  A pinned curve
//!   parameter is `==` precisely because it changes the solution set: without the pin, a curve
//!   fitted through m points keeps m degrees of freedom.
//!
//! That distinction is lexical, which is what makes "may a solve write this number?" a test
//! rather than an analysis — see `edit::commit_seeds`.

use crate::constraints::{is_operator, CKind, Fixity, SpecKind};
use crate::model::{EntKind, EntRef, Field};
use crate::style::{Classes, Style};

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

    /// What a formal declared this way *is* (`units.rs`) — the same table one question further
    /// on, and here for `parse`'s reason: a second copy of the list would be a second answer the
    /// moment a type is added.  `Length` and `Angle` name the two base dimensions; `Int` and
    /// `Scalar` are plain numbers, and an entity formal is not a number at all.
    pub fn dim(self) -> crate::units::Dim {
        match self {
            Ty::Length => crate::units::Dim::LENGTH,
            Ty::Angle => crate::units::Dim::ANGLE,
            Ty::Int | Ty::Scalar | Ty::Ent(_) => crate::units::Dim::SCALAR,
        }
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
    /// A worded joint that also threads: `… -> tangent …`.  Doomed, it steps down to the bare
    /// corner `->` — the claim goes, and the corner stays.
    Joint,
    /// A worded joint that does not thread — an infix relation between two links, `… equal …`.
    /// Its span was chosen at desugar time to be deletable (the word, plus a terminal name-link
    /// a deletion must take with it); doomed, the span becomes a statement break.
    Infix,
    /// A worded joint no splice can remove: it stands unthreaded in a chain that closes, where
    /// a break would re-aim the `close` at another link.  Deleting it is refused, the link's
    /// own bargain.
    Stuck,
    /// One of the several words a joint may state, `-> equal angle(30deg)`: doomed, it
    /// splices out where it stands, and the corner and the joint's other statements stand.
    /// The whole joint doomed at once — an entity deletion dooms every relation naming it —
    /// has no word left to hold its place, so each member carries the joint's written word
    /// count and what its *only* word's doom would be, over the joint's own span, and
    /// `edit::doomed_splices` composes that one splice when all `out_of` fall together.
    Member { of: Span, fall: Fall, out_of: u32 },
    /// The joint before `close`, which seals a loop.
    Close,
}

/// What a joint's *only* word's doom is — the spelling a whole run of words falls back to
/// when every one of them is doomed at once, carried by each `Chained::Member`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fall {
    /// threaded: the corner outlives its words
    Joint,
    /// unthreaded: the words were the statement, and a break takes their place
    Infix,
    /// unthreaded in a chain that closes: no break is safe, so the whole is refused
    Stuck,
    /// the loop-sealing joint: `-> close` outlives its words
    Close,
}

impl From<Fall> for Chained {
    fn from(f: Fall) -> Chained {
        match f {
            Fall::Joint => Chained::Joint,
            Fall::Infix => Chained::Infix,
            Fall::Stuck => Chained::Stuck,
            Fall::Close => Chained::Close,
        }
    }
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
    /// `style .construction { dash: 7 4 }` — what a class looks like.  Presentation, and the
    /// one statement that says nothing about what the drawing *is*.
    Style(StyleRule),
    /// `unit mm` — what the document's numbers are in (spec §3.3).  A bare number in a `Length`
    /// slot is that unit, so every document keeps working with one added line; a document that
    /// says nothing is in **drawing units**, and everything still dimension-checks, you simply
    /// cannot write `mm` because there is nothing to convert to.
    Unit(Name),
}

/// `style .NAME { prop: value; … }`.
///
/// The properties are held as a resolved `Style` rather than as text: the little language they
/// are written in is three keywords and a number, so unlike a dimension there is nothing here a
/// second reader could disagree about.  An unknown property is dropped with a warning span, the
/// way an unmatched class is simply not a rule.
#[derive(Clone, Debug)]
pub struct StyleRule {
    pub name: Name,
    pub style: Style,
    /// The property names as written, in order, so the printer says what the source said.
    pub props: Vec<String>,
    pub span: Span,
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

/// One `name: value` of a `hint(…)` clause, read but not yet resolved: the key, where it was
/// written, and the value exactly as `value_text` gave it back.
struct Hint {
    key: String,
    /// Where the key stands, so an unknown one is reported there rather than after its value.
    at: Span,
    value: Option<f64>,
    text: String,
    span: Span,
}

impl From<Hint> for OpArg {
    /// A hint read from a `hint(…)` clause is a **seed** for the slot it names — the only thing
    /// such a clause can be (spec §4.3).  A pin is the same argument with `pinned` set, built
    /// where `==` is read instead.
    fn from(h: Hint) -> OpArg {
        OpArg::Slot {
            key: Name { text: h.key, span: h.at },
            arg: seed_arg(h.value, h.text, h.span, false),
        }
    }
}

/// `point p0 hint(x: 0, y: 0)`, `circle c0(center: p2) hint(r: 25)`,
/// `spline s0(p3, p4, p5, p6) knots [...]`.
#[derive(Clone, Debug)]
pub struct Decl {
    pub kind: EntKind,
    pub name: Name,
    /// Whether the **source** wrote that name.  An anonymous declaration carries one all the
    /// same — `#a` and its own offset, so a chain's corner and the desugared statements have
    /// something to resolve by — but a key is not a name, and this is the *only* place the
    /// difference is known first-hand: the parser either took an identifier or declined to.
    /// Every later reader is told (`SourceMap::bind`, `program::shown`, `edit::reconcile`)
    /// rather than asking the characters, which could tell a key from a name only by the `#a`
    /// the mint happens to use.
    ///
    /// A prefix the flattener puts on the front does not touch it: `#5.0.p0` is a name the
    /// source wrote, said in an instance's terms.
    pub named: bool,
    /// One per `Child`/`List` field of `EntKind::fields`, in that order; a `List` field holds as
    /// many as were written.  Empty throughout — `line l` — is the anonymous form: the kind's
    /// children are minted, unnamed, and reached as `l.p1`.
    pub children: Vec<Vec<Kid>>,
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
    /// The `hint(…)` clause: where it *is* when the statement wrote one, and where it *would go*
    /// — an empty span at the insertion point — when it did not.  `None` for a declaration that
    /// was built rather than parsed.
    ///
    /// Both cases at once, because writeback needs both and one of them is not a special case:
    /// a solve moves a scalar the document never wrote (a radius, a frame's rotor), and there
    /// has to be somewhere to record it.  `edit::commit_seeds` splices the individual seeds when
    /// every one it needs already has a span, and rewrites this whole clause when one does not.
    pub hint_span: Option<Span>,
    /// Document data no solve moves, so not a seed and never written back.
    pub knots: Option<Vec<f64>>,
    /// A curve instance: the family it belongs to.  `None` for every other kind.
    pub def: Option<Name>,
    /// The numbers a curve instance is given, as written.
    pub values: Vec<(Name, String)>,
    /// The interval a curve instance is drawn over, as written.
    pub domain: Option<(String, String)>,
    /// The classes it carries, in written order: `line l(a, b) class centerline heavy`.
    /// Presentation, and nothing the core computes reads it (spec §14).
    pub class: Classes,
    /// Where `class …` sits in the source, so a toggle rewrites the words and not the statement
    /// around them.  An *empty* span at the point one would be written when there is none.
    pub class_span: Span,
    /// A seed named *geometrically* rather than by coordinates: `at t`, `at c.center`,
    /// `at c bearing (u + phase)`.  What it may name is the elaborator's question.
    pub seed_at: Option<AtRef>,
}

/// What fills one child slot of a declaration.
///
/// A written slot carries a **name** or a **seed**, and there is no third form: `line l(a, b)`
/// names its ends and `line l(hint(x: 0, y: 0), hint(x: 60, y: 20))` seeds two points nothing
/// names.  An entity whose children are all unnamed and unseeded is spelled by writing no list
/// at all — `line l` — which is why "anonymous and unseeded" needs no spelling of its own.
///
/// The one place a *partial* list exists is mid-desugaring, where a chain's joint has not yet
/// filled the boundary slot the link left out (§6.6); it is filled before elaboration sees it.
#[derive(Clone, Debug)]
pub enum Kid {
    /// `line l(a, b)` — the point is named, and named somewhere else.
    Ref(Ref),
    /// `line l(hint(x: 0, y: 0), …)` — an anonymous point, and where its solve begins.  The
    /// same clause as everywhere else in the language, one level down.
    Hint(KidSeed),
}

impl Kid {
    pub fn as_ref(&self) -> Option<&Ref> {
        match self {
            Kid::Ref(r) => Some(r),
            Kid::Hint(_) => None,
        }
    }
}

/// The seed inside a child slot: an anonymous point's `x` and `y`, carried exactly as
/// `Decl::seed` / `seed_text` / `seed_spans` carry an entity's own scalars, and for the same
/// reasons — a solve splices the numbers and never the words around them.
#[derive(Clone, Debug, Default)]
pub struct KidSeed {
    pub v: [f64; 2],
    /// As written, where it was written as an expression over the parameters in scope.
    pub text: [Option<String>; 2],
    /// Where each number sits in the source.
    pub spans: [Span; 2],
    /// The whole `hint(…)`, so a writeback that has to add a key can rewrite it.
    pub span: Span,
}

/// `at c bearing (u + phase)` — a place given as geometry: at a point, or at the edge of a
/// circle at a bearing from the page's x-axis.
#[derive(Clone, Debug)]
pub struct AtRef {
    pub what: Ref,
    pub bearing: Option<(String, Span)>,
}

/// What stood in an operator's parentheses.
///
/// Everything that is not one of the two operands, and it is a short list: the number, a
/// selector, and — for `symmetry`, `ccw` and `cw` — a third entity.
#[derive(Clone, Debug)]
pub enum OpArg {
    /// `side: -1`, `at: start`, `along: x`, `external: true`
    Named(Name, Arg),
    /// the third entity, unlabelled: `a symmetry(l) b`
    Ent(Ref),
    /// A slot the constraint owns, **named as the spec names it** — `t == 0.4` in the
    /// parentheses, or `hint(t: 0.4)` after the operands.
    ///
    /// One variant, because the word is the whole of the difference: a **pin** is a stated
    /// number the solve may not revise and a **seed** is where it begins (spec §4.3), and both
    /// are the same number read the same way — a literal, or an expression over the parameters
    /// in scope (`hint(u: u0)` inside a component), worked out during expansion.  Which of the
    /// two, and which of literal and expression, is `Arg`'s to say and not a second encoding
    /// here: this is `Arg::Seed`/`Arg::SeedExpr` with the key in front, so `assemble` hands the
    /// value straight on and `flatten` settles it through the one walk it already had.
    ///
    /// The key is a `Name` and carries **its own span** — the key is what an unknown-slot
    /// message is about, so the caret belongs on it and not on the value after it, which is the
    /// rule `kid_seed` and the declaration's own clause already keep.  It is kept at all because
    /// a kind's slot is `t` on a spline and `u` on a curve, and neither the check in
    /// `Written::assemble` nor the printer may guess which.
    Slot { key: Name, arg: Arg },
    /// the number, as written — `80`, `x = 7`, `h = w / 2`, `1' 3"`
    Dim(String, Span),
}

/// A constraint statement **as it was written**: an operator, its operands, and what stood in
/// its parentheses (spec §9.1).
///
/// `name(args…)` is retired.  What a word means depends on the *kinds* of its operands — `on` is
/// five constraints, `distance` is six, `tangent` is six — and a name's kind is not known until
/// elaboration, so this is what the parser produces and `program::constrain` is what settles it.
/// One path, where 0.6 had a longhand and a chain.
#[derive(Clone, Debug)]
pub struct Written {
    pub word: Name,
    pub fixity: Fixity,
    /// One (prefix) or two (infix), in written order.  **Order carries meaning**: `arc tangent
    /// line` is `TangentArcLine` and `line tangent circle` is `TangentLineCircle`.
    pub ops: Vec<Ref>,
    pub args: Vec<OpArg>,
    pub span: Span,
}

impl Written {
    /// One selector by name — what `constraints::infix_op` reads to tell `distance … along: x`
    /// from a plain one, and a tangency at a named end from the bare pair.
    pub fn sel(&self, name: &str) -> Option<String> {
        self.args.iter().find_map(|a| match a {
            OpArg::Named(n, v) if n.text == name => Some(match v {
                Arg::Word(w) => w.clone(),
                Arg::Int(i) => i.to_string(),
                Arg::Bool(b) => b.to_string(),
                Arg::Num(x) => num(*x),
                _ => String::new(),
            }),
            _ => None,
        })
    }

    /// The arguments a constraint of `kind` takes, in **spec order**, from what was written.
    ///
    /// The one place the operator form becomes the library's form, so the parser and the
    /// elaborator cannot disagree about which slot a selector filled.  Entity slots come first
    /// in spec order and are filled from the operands, then from an unlabelled entity in the
    /// parentheses (`symmetry`'s line); the trailing dimension from the number; a `Param` from
    /// the pin; and everything else by its own name.
    pub fn assemble(&self, kind: CKind) -> Result<Vec<Option<Arg>>, (Span, String)> {
        let spec = kind.spec();
        // a seed or a pin names the slot it fills, and a name the kind does not have is a typo
        // rather than something to fill the first slot with: `on` owns `t` on a spline and `u`
        // on a curve, so the wrong word here would silently pin the right slot at the wrong
        // number.  Checked before anything is assembled, so the message is about what was
        // written and not about what it came to.
        for a in &self.args {
            let OpArg::Slot { key, .. } = a else { continue };
            if !spec.iter().any(|(n, k)| k.is_param() && *n == key.text) {
                let word = &self.word.text;
                let m = format!("`{word}` has no slot `{}` to seed", key.text);
                return Err((key.span, m));
            }
        }
        let mut out: Vec<Option<Arg>> = vec![None; spec.len()];
        let mut ents: Vec<Ref> = self.ops.clone();
        // `distance line1` is the distance between the line's own ends — the one prefix word
        // that is sugar for a statement about something else's parts
        if kind == CKind::Distance && self.ops.len() == 1 {
            let r = &self.ops[0];
            ents = ["p1", "p2"]
                .iter()
                .map(|f| Ref {
                    root: r.root.clone(),
                    path: vec![Seg::Field(Name::new(*f))],
                    span: r.span,
                })
                .collect();
        }
        ents.extend(self.args.iter().filter_map(|a| match a {
            OpArg::Ent(r) => Some(r.clone()),
            _ => None,
        }));
        let mut next = ents.into_iter();
        for (i, (name, sk)) in spec.iter().enumerate() {
            out[i] = if sk.is_entity() {
                next.next().map(Arg::Ref)
            } else if sk.is_param() {
                self.args.iter().find_map(|a| match a {
                    OpArg::Slot { key, arg } if key.text == *name => Some(arg.clone()),
                    _ => None,
                })
            } else if sk.is_dimension() && i + 1 == spec.len() {
                self.args.iter().find_map(|a| match a {
                    OpArg::Dim(t, sp) => Some(Arg::Dim { text: t.clone(), span: *sp }),
                    _ => None,
                })
            } else {
                self.args.iter().find_map(|a| match a {
                    OpArg::Named(n, v) if n.text == *name => Some(v.clone()),
                    _ => None,
                })
            };
        }
        Ok(out)
    }
}

/// A constraint statement: `p0 distance(80) p1 at (12, -4)`.
#[derive(Clone, Debug)]
pub struct Relation {
    pub kind: CKind,
    /// One per `CKind::spec()` slot; `None` where the source left an inferred slot out.
    pub args: Vec<Option<Arg>>,
    /// Where the callout was dragged to, if anywhere.  A seed: inert, and written back.
    pub place: Option<(f64, f64)>,
    /// Where `at (t, r)` sits in the source, so a callout dragged somewhere else rewrites those
    /// characters instead of the statement around them.  Empty for a relation that was built
    /// rather than parsed, and for one that carries no placement — in both cases there is no
    /// text yet, and the writeback appends after the statement.
    pub place_span: Span,
    /// The statement **as it was written**, where it was written as an operator — which is every
    /// statement a document contains, since `name(args…)` is retired (spec §9.1).
    ///
    /// The word alone does not say which constraint it is: `on` is five, `distance` is six, and
    /// what tells them apart is the *kinds* of the operands, which a name does not carry until
    /// elaboration.  So the word and its parentheses travel, `program::constrain` settles them,
    /// and `kind`/`args` are a placeholder until it does.  `None` is a relation somebody
    /// **built** rather than wrote — `edit::add_relation`, `program::lift_relation` — where the
    /// kind is known from the start and the printer works the operator out backwards.
    pub poly: Option<Written>,
    /// Written `claim …` (§9.7): stated as expected to add no rank, judged by the diagnosis and
    /// never solved for.
    pub claim: bool,
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
        // the gauges are prefix operators like every other statement (spec §9.1)
        StmtKind::Gauge(Gauge::Ground(r)) => {
            out.push_str("ground ");
            write_ref(out, r);
        }
        StmtKind::Gauge(Gauge::Fix(r)) => {
            out.push_str("fix ");
            write_ref(out, r);
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
        StmtKind::Unit(n) => out.push_str(&format!("unit {}", n.text)),
        StmtKind::Style(r) => {
            out.push_str(&format!("style .{} {{ ", r.name.text));
            let parts: Vec<String> = r
                .props
                .iter()
                .filter_map(|k| style_prop_text(&r.style, k).map(|v| format!("{k}: {v}")))
                .collect();
            out.push_str(&parts.join("; "));
            out.push_str(" }");
        }
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
    // an anonymous declaration has no name to spell — its `#` key is the elaboration's, not
    // the source's — so the keyword stands alone and the tail glues to it
    if !hidden(&d.name.text) {
        for _ in kw.len()..KW {
            out.push(' ');
        }
        out.push(' ');
        out.push_str(&d.name.text);
    }

    out.push_str(&decl_tail(d, &d.seed));
    if let Some(u) = &d.knots {
        out.push_str(" knots [");
        out.push_str(&u.iter().map(|&v| num(v)).collect::<Vec<_>>().join(", "));
        out.push(']');
    }
    if !d.class.is_empty() {
        out.push_str(" class ");
        out.push_str(&d.class.0.join(" "));
    }
}

/// One property of a style, as a `style` block writes it.
fn style_prop_text(s: &Style, prop: &str) -> Option<String> {
    match prop {
        "dash" => s.dash.as_ref().map(|d| d.iter().map(|&v| num(v)).collect::<Vec<_>>().join(" ")),
        "width" => s.width.map(num),
        "color" => s.color.clone(),
        _ => None,
    }
}

/// `(center: p2)` — what a declaration says the thing is *made of*, or nothing when it names
/// none of it.  A slot holds a name or a seed, and a seed is the same `hint(…)` clause it is
/// everywhere else, one level down.
pub(crate) fn decl_args(d: &Decl) -> String {
    // A *gap* in the slots forces labels on: an unlabelled child counts into its slot by
    // position, so where an earlier slot stands empty — a corner the writeback left for the
    // chain's marker to thread again — a bare `line(hint(…))` would put the kept end in the
    // wrong slot on the next parse, and the pose committed for it would quietly reseed.
    let after_gap = d
        .children
        .iter()
        .rposition(|g| !g.is_empty())
        .is_some_and(|k| d.children[..k].iter().any(|g| g.is_empty()));
    let label = labels_children(d.kind) || after_gap;
    let mut parts: Vec<String> = Vec::new();
    let mut child = 0usize;
    for (name, field) in d.kind.fields() {
        match field {
            Field::Child | Field::List => {
                let kids = d.children.get(child).map(|g| g.as_slice()).unwrap_or_default();
                child += 1;
                for k in kids {
                    let mut s = String::new();
                    if label && *field == Field::Child {
                        s.push_str(name);
                        s.push_str(": ");
                    }
                    match k {
                        Kid::Ref(r) => write_ref(&mut s, r),
                        Kid::Hint(k) => s.push_str(&kid_seed_text(k)),
                    }
                    parts.push(s);
                }
            }
            // every scalar is a seed, and every seed is in the `hint(…)` clause: the brackets
            // after the name are what the thing is *made of*, and a radius is not that
            Field::Scalar => {}
        }
    }
    if parts.is_empty() {
        String::new()
    } else {
        format!("({})", parts.join(", "))
    }
}

/// Everything a declaration says after its name bar the order-free trailers: what it is made of,
/// and where its solve begins.
///
/// One function because `edit::commit_seeds` writes this same tail when a solve moved a number
/// the source never wrote, and a statement printed two ways is two spellings of one clause —
/// which is how the writeback came to drop the `center:` a printed `arc` puts in.  No leading
/// space, for `hint_of`'s reason: the separator belongs to whoever is joining the statement up.
pub(crate) fn decl_tail(d: &Decl, seed: &[f64]) -> String {
    let mut out = decl_args(d);
    let hint = hint_clause(d, seed);
    if !hint.is_empty() {
        // the list glues to the name; the clause is a word of its own and brings its separator,
        // whether it follows the name or the bracket
        out.push(' ');
        out.push_str(&hint);
    }
    out
}

/// `hint(a: 1, b: 2)` from its parts, or nothing at all when there are none.
///
/// No leading space: the separator belongs to whatever is joining the statement up, which is
/// the one place that knows whether anything came before — a splice into a gap does not.  The
/// printed clause has one spelling, so it has one place that spells it.
/// What a `Param` slot's number comes to, seeded or pinned: a literal, or an expression over the
/// parameters in scope with the span an edit would splice it at.  Which of the two it is, is the
/// word, and the word is this one flag.
fn seed_arg(value: Option<f64>, text: String, span: Span, pinned: bool) -> Arg {
    match value {
        Some(value) => Arg::Seed { value, pinned },
        None => Arg::SeedExpr { text, pinned, span },
    }
}

fn hint_of(parts: &[String]) -> String {
    if parts.is_empty() {
        String::new()
    } else {
        format!("hint({})", parts.join(", "))
    }
}

/// `hint(x: 0, y: 0)` — a point's scalars, keyed by the names `fields()` gives them.
///
/// The keys come off the one table so that the parser, the printer and the writeback cannot
/// disagree about what a point's coordinates are called, which is the reason `fields()` exists.
fn point_hint(text: [Option<&str>; 2], v: [f64; 2]) -> String {
    let parts: Vec<String> = EntKind::Point
        .fields()
        .iter()
        .filter(|(_, f)| *f == Field::Scalar)
        .enumerate()
        .map(|(i, (name, _))| match text.get(i).copied().flatten() {
            Some(t) => format!("{name}: {t}"),
            None => format!("{name}: {}", num(v.get(i).copied().unwrap_or(0.0))),
        })
        .collect();
    hint_of(&parts)
}

/// The same, for a place a solve arrived at: numbers, and no text anybody wrote.
pub(crate) fn hint_xy(x: f64, y: f64) -> String {
    point_hint([None, None], [x, y])
}

/// `hint(x: 0, y: 0)` standing in a child slot — an anonymous point, and where its solve begins.
pub(crate) fn kid_seed_text(k: &KidSeed) -> String {
    point_hint([k.text[0].as_deref(), k.text[1].as_deref()], k.v)
}

/// `hint(x: 0, y: 0)` — every scalar the kind owns, keyed by the name `fields()` gives it.
///
/// All of them or none: a kind with no scalars writes nothing, and a kind with some writes them
/// whatever they are, so the printed form round-trips without a rule about which numbers are
/// worth saying.  A declaration whose seed is a *place* (`hint at t`) has no coordinates to
/// write and is left to the source it came from — so an empty string is also the answer to "is
/// there a clause to write here at all?", which is the only test a caller needs.
///
/// The numbers are passed rather than read off `d`, because `edit::commit_seeds` prints the pose
/// a solve just arrived at and the declaration still holds the one it started from.
pub(crate) fn hint_clause(d: &Decl, seed: &[f64]) -> String {
    if d.seed_at.is_some() {
        return String::new();
    }
    let mut parts: Vec<String> = Vec::new();
    let mut scalar = 0usize;
    for (name, field) in d.kind.fields() {
        if *field != Field::Scalar {
            continue;
        }
        let v = seed.get(scalar).copied().unwrap_or(0.0);
        let text = match d.seed_text.get(scalar).and_then(|t| t.as_deref()) {
            Some(t) => t.to_string(),
            None => num(v),
        };
        scalar += 1;
        parts.push(format!("{name}: {text}"));
    }
    hint_of(&parts)
}

fn write_relation(out: &mut String, r: &Relation) {
    if r.claim {
        out.push_str("claim ");
    }
    // what somebody wrote, where they wrote it: a statement waiting on elaboration prints back
    // as its operator, and the kind beside it is a placeholder that has not been settled yet
    if let Some(w) = &r.poly {
        write_written(out, w);
        if let Some((t, rr)) = r.place {
            out.push_str(&format!(" at ({}, {})", num(t), num(rr)));
        }
        return;
    }
    out.push_str(&operator_text(r.kind, &r.args));
    if let Some((t, rr)) = r.place {
        out.push_str(&format!(" at ({}, {})", num(t), num(rr)));
    }
}

fn write_written(out: &mut String, w: &Written) {
    let mut parts: Vec<String> = Vec::new();
    let mut hints: Vec<String> = Vec::new();
    for a in &w.args {
        match a {
            OpArg::Named(n, v) => parts.push(format!("{}: {}", n.text, sel_text(v))),
            OpArg::Ent(r) => {
                let mut s = String::new();
                write_ref(&mut s, r);
                parts.push(s);
            }
            OpArg::Dim(t, _) => parts.insert(0, t.clone()),
            // the slot's own name, as it was written: `t` on a spline and `u` on a curve.  The
            // same `slot_text` `operator_text` reads it off the spec with, so the two printers
            // cannot come to spell one slot differently.
            OpArg::Slot { key, arg } => match slot_text(&key.text, arg) {
                Some((true, t)) => parts.push(t),
                Some((false, t)) => hints.push(t),
                None => {}
            },
        }
    }
    let head = |out: &mut String, r: &Ref| {
        write_ref(out, r);
        out.push(' ');
    };
    if w.fixity == Fixity::Infix {
        if let Some(l) = w.ops.first() {
            head(out, l);
        }
    }
    out.push_str(&w.word.text);
    if !parts.is_empty() {
        out.push_str(&format!("({})", parts.join(", ")));
    }
    let last = if w.fixity == Fixity::Infix { w.ops.get(1) } else { w.ops.first() };
    if let Some(r) = last {
        out.push(' ');
        write_ref(out, r);
    }
    let hint = hint_of(&hints);
    if !hint.is_empty() {
        out.push(' ');
        out.push_str(&hint);
    }
}

/// A constraint the *library* holds, written as the operator it is spelled with (spec §9.1).
///
/// The inverse of parsing, and the one place it is done — `write_relation` prints a statement
/// somebody built rather than wrote, and `io::describe` prints one for a reader.  So the drawing,
/// the constraint list and the program panel cannot come to spell one constraint three ways.
pub fn operator_text(kind: CKind, args: &[Option<Arg>]) -> String {
    let Some((word, fixity)) = kind.operator() else {
        // nobody writes this one: a drag target, a frame's intrinsics
        return format!("{}(…)", snake(kind.name()));
    };
    let spec = kind.spec();
    let mut ents: Vec<String> = Vec::new();
    let mut parens: Vec<String> = Vec::new();
    let mut hints: Vec<String> = Vec::new();
    // which of the three a pair of points means is not in the kind's name but in `along:`
    match kind {
        CKind::HorizontalDistance => parens.push("along: x".to_string()),
        CKind::VerticalDistance => parens.push("along: y".to_string()),
        _ => {}
    }
    for (i, (name, sk)) in spec.iter().enumerate() {
        let Some(a) = args.get(i).and_then(|a| a.as_ref()) else { continue };
        if sk.is_entity() {
            ents.push(write_arg(name, *sk, a));
        } else if sk.is_param() {
            match slot_text(name, a) {
                Some((true, t)) => parens.push(t),
                Some((false, t)) => hints.push(t),
                None => {}
            }
        } else if sk.is_dimension() && i + 1 == spec.len() {
            parens.insert(0, dim_text(a));
        } else {
            parens.push(write_arg(name, *sk, a));
        }
    }
    // the third entity of `symmetry` goes in the parentheses with everything else that is not
    // one of the two operands
    while ents.len() > 2 {
        let extra = ents.pop().expect("more than two");
        parens.push(extra);
    }
    let mut out = String::new();
    if fixity == Fixity::Infix && !ents.is_empty() {
        out.push_str(&ents.remove(0));
        out.push(' ');
    }
    out.push_str(word);
    if !parens.is_empty() {
        out.push_str(&format!("({})", parens.join(", ")));
    }
    for e in &ents {
        out.push(' ');
        out.push_str(e);
    }
    let hint = hint_of(&hints);
    if !hint.is_empty() {
        out.push(' ');
        out.push_str(&hint);
    }
    out
}

/// One owned slot, written down: `Some((true, "t == 0.4"))` for the parentheses, where a **pin**
/// is a stated number beside every other stated number, and `Some((false, "t: 0.4"))` for the
/// `hint(…)` clause, where every seed in the language is (spec §4.3).
///
/// **The one place that spells a slot**, asked by `operator_text` off the spec and by
/// `write_written` off the key the document used — the same bargain `hint_of` strikes with the
/// clause around it.  `None` is a slot that states no number at all: a `Param` the sketch has
/// already allocated, which `describe` leaves out because it is the solver's business.
fn slot_text(name: &str, a: &Arg) -> Option<(bool, String)> {
    let (pinned, v) = match a {
        Arg::Seed { value, pinned } => (*pinned, num(*value)),
        Arg::SeedExpr { text, pinned, .. } => (*pinned, text.clone()),
        _ => return None,
    };
    Some(match pinned {
        true => (true, format!("{name} == {v}")),
        false => (false, format!("{name}: {v}")),
    })
}

/// A selector's value, as a `style` block writes one.
fn sel_text(a: &Arg) -> String {
    match a {
        Arg::Num(v) => num(*v),
        Arg::Int(v) => v.to_string(),
        Arg::Bool(b) => b.to_string(),
        Arg::Word(w) => w.clone(),
        other => format!("{other:?}"),
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
        // only a *pin* reaches here: an unpinned slot is a seed and `write_relation` puts it in
        // the `hint(…)` clause, which is where every seed in the language is written
        Arg::Seed { value, .. } => format!("{name} == {}", num(*value)),
        Arg::Num(v) if sk == SpecKind::Angle => format!("{name}: {}", num(v.to_degrees())),
        Arg::Num(v) => format!("{name}: {}", num(*v)),
        Arg::Int(v) => format!("{name}: {v}"),
        Arg::Bool(b) => format!("{name}: {b}"),
        // **The language has no string literal** — a quote is the inch mark (spec §3.3) — so a
        // `Str` slot is written as the word it is (`at: start`), bare.
        Arg::Word(w) => format!("{name}: {w}"),
        Arg::Dim { text, .. } => format!("{name}: {text}"),
        Arg::SeedExpr { text, .. } => format!("{name} == {text}"),
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
        out.push_str(&format!("branch({key}, {v})"));
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
    /// `(` `)` `[` `]` `,` `:` `.` `-` `{` `}`
    P(char),
    /// `=` — a seed
    Eq,
    /// `==` — a constraint
    EqEq,
    /// `->` — a chain's joint marker: the two links beside it share a boundary point (§6.6)
    Arrow,
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
            // `->` is the joint marker (§6.6).  `>` is not a token on its own, so the pair is
            // claimed here before `-` can read as punctuation.
            '-' if b.get(i + 1) == Some(&b'>') => {
                i += 2;
                toks.push((Tok::Arrow, Span::new(lo, i)));
            }
            // `'` and `"` are the foot and inch marks (spec §3.3), and `|` is what a raw
            // branch key separates its points with.  The language has **no string literal**:
            // a quote after a number is a unit, and there is nothing else for one to be.
            '(' | ')' | '[' | ']' | ',' | ':' | '{' | '}' | '-' | '+' | '\'' | '"' | '|' => {
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
    /// `class centerline`, and the `.centerline` a `style` block names — presentation, which is
    /// a different statement from what the drawing is and reads as one
    Class,
}

impl Tint {
    /// The class a front end styles it by.  Named here so the core says what the colours are *of*
    /// and a stylesheet only says what they look like.
    pub fn as_str(self) -> &'static str {
        match self {
            Tint::Comment => "comment",
            Tint::Num => "number",
            Tint::Word => "word",
            Tint::Type => "type",
            Tint::Relation => "relation",
            Tint::Def => "def",
            Tint::Label => "label",
            Tint::Seed => "seed",
            Tint::Claim => "claim",
            Tint::Class => "class",
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

/// The words that open a statement of their own, so a name may never be one.  Written down here
/// because an infix statement begins with a *name*, and a keyword followed by a word that
/// happens to be an operator — `param radius = 50` — would otherwise read as one.
const OPENERS: [&str; 12] = [
    "claim", "component", "param", "port", "unit", "style", "branch", "repeat", "cycle", "ring",
    "ground", "fix",
];
const ORIENTS: [&str; 2] = ["ccw", "cw"];
const BLOCKS: [&str; 3] = ["repeat", "cycle", "ring"];

/// Whether a word may stand between two links of a chain (spec §6.6).  `tangent` is the
/// drafting word, mapped per pair of kinds to the regular At-form where the joint threads; and
/// any binary constraint whose spec is exactly two entity slots — `perpendicular`,
/// `equal_length`, `equal_radius` — is an infix spelling of itself, the two-argument counterpart
/// of `prefix_kind` and derived from the same registry.  `equal` is the polymorphic one
/// (`equal_kind`).  The plain corner is not a word at all but the `->` marker, and `close`,
/// which seals a loop, is not a joint — it stands where a link would.
fn joint_word(w: &str) -> bool {
    is_operator(w)
}

/// The words that shape a statement without naming anything — a modifier the parser eats where it
/// stands.  `as` binds a name after it, which is why `highlight` treats that one specially.
const MODIFIERS: [&str; 9] =
    ["over", "as", "at", "hint", "about", "class", "where", "bearing", "from"];

/// The words that may follow a declaration's own, so `class a b` knows where its list ends.
/// A chain's joints are here too: `arc a(center: c) class construction tangent …` is one link.
const TRAILERS: [&str; 4] = ["knots", "hint", "class", "close"];

/// What the word *after* this one is expected to be — the whole of the state the colouring carries
/// from one token to the next, and four states rather than the four independent flags that would
/// spell out twelve combinations the language never reaches.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Next {
    /// begins a statement, so it is the word that says what the statement *is*
    Start,
    /// names a class: every plain word after `class`, and the one a `style` block declares
    Class,
    /// names the document's unit: the one word after `unit`
    Unit,
    /// names what the statement declares
    Def,
    /// the same, after an element keyword — where the name is *optional* (§6.1), so a word
    /// reserved for what may follow a declaration keeps its own reading instead
    DeclName,
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
    // how deep inside a `hint(…)` clause we are, and 0 outside one.  A number inside one is a
    // seed and every other number is not — the whole of §4.3's lexical rule, and the reason the
    // colouring can say which numbers a solve may rewrite without elaborating anything.
    let mut hint = 0i32;
    // and how deep inside a `style .name { … }` block, whose body is `property: value` pairs
    // rather than statements
    let mut style = 0i32;
    for (i, (t, span)) in lexed.toks.iter().enumerate() {
        let prev = i.checked_sub(1).map(|j| &lexed.toks[j].0);
        match t {
            Tok::P('(') if matches!(prev, Some(Tok::Ident(w)) if w == "hint") => hint = 1,
            Tok::P('(') | Tok::P('[') if hint > 0 => hint += 1,
            Tok::P(')') | Tok::P(']') if hint > 0 => hint -= 1,
            Tok::Nl => hint = 0,
            _ => {}
        }
        if matches!(t, Tok::Ident(w) if w == "style") && at == Next::Start {
            style = 1;
        } else if matches!(t, Tok::P('}')) && style > 0 {
            style = 0;
        }
        let tint = match t {
            Tok::Nl => {
                at = Next::Start;
                continue;
            }
            Tok::Num(_) if hint > 0 => Some(Tint::Seed),
            Tok::Num(_) => Some(Tint::Num),
            // `param w = 100` and `curve e = involute(…)` — an assignment, and not a seed: the
            // clause is what says a number is one now
            Tok::Eq => None,
            Tok::EqEq => Some(Tint::Claim),
            // the joint marker is structure, the way `close` is
            Tok::Arrow => Some(Tint::Word),
            // a body is made of statements, so a brace begins one the way a newline does
            Tok::P('{') | Tok::P('}') => {
                at = Next::Start;
                None
            }
            Tok::P(_) => None,
            Tok::Ident(w) => {
                let (tint, then) = tint_word(w, prev, &lexed.toks, i, at);
                at = then;
                tint
            }
        };
        // anything at all leaves the opening word behind; a name the statement is still owed
        // (`Def`, `Inst`) survives the punctuation in between, which is why only `Start` lapses
        if at == Next::Start && !matches!(t, Tok::P('{') | Tok::P('}')) {
            at = Next::Word;
        }
        // a declaration's name stands against its keyword or not at all — the name is optional,
        // so `line(p1, p2)` must not read `p1` as one once the bracket has gone by
        if at == Next::DeclName && !matches!(t, Tok::Ident(_)) {
            at = Next::Word;
        }
        // a `style` block's body is `property: value` pairs, not statements: the brace above
        // reset the state to `Start`, where `dash:` would read as an instance
        if matches!(t, Tok::P('{')) && style > 0 {
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
fn tint_word(
    w: &str,
    prev: Option<&Tok>,
    toks: &[(Tok, Span)],
    i: usize,
    at: Next,
) -> (Option<Tint>, Next) {
    let next = toks.get(i + 1).map(|(t, _)| t);
    match at {
        Next::Unit => (Some(Tint::Type), Next::Word),
        Next::Class => {
            // the list runs to the next thing a declaration may say — another trailing clause,
            // or a chain's joint.  The same predicate the parser stops on, asked once.
            if trails_decl(w) {
                return tint_word(w, prev, toks, i, Next::Word);
            }
            (Some(Tint::Class), Next::Class)
        }
        Next::Def => (Some(Tint::Def), Next::Word),
        Next::DeclName => {
            // the name is optional: what follows the keyword may be the next thing the
            // statement says — a clause, a joint, the next link — and those words keep their
            // own colour.  The same predicate the parser decides by, asked once.
            if !names_decl(w) {
                return tint_word(w, prev, toks, i, Next::Word);
            }
            (Some(Tint::Def), Next::Word)
        }
        Next::Inst => (Some(Tint::Type), Next::Word),
        Next::Start => {
            // `point p`, `component Gear(…)`, `curve involute(…)`, `param R = …`, `port lo: point`
            if EntKind::parse(w).is_some() {
                return (Some(Tint::Word), after_kind(w));
            }
            if matches!(w, "component" | "param" | "port") {
                return (Some(Tint::Word), Next::Def);
            }
            // `style .construction { … }` — the class it names is the thing it declares
            if w == "style" {
                return (Some(Tint::Word), Next::Class);
            }
            // `unit mm` — the word, and the unit it names (spec §3.3)
            if w == "unit" {
                return (Some(Tint::Word), Next::Unit);
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
            // `claim vertical(rail)`: the word after it is a statement start again, so the
            // relation it qualifies is tinted exactly as it would be standing alone
            if w == "claim" {
                return (Some(Tint::Word), Next::Start);
            }
            // the operator table, not the registry's names: `on` and `equal` are constraints
            // the language writes and are not any `CKind`'s name, and `point_on_circle` is a
            // name no document writes any more (spec §9.1)
            (is_operator(w).then_some(Tint::Relation), Next::Word)
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
                // `cycle N as i` — the binder is a name the block declares; `class a b` names
                // classes until the clause ends
                return (
                    Some(Tint::Word),
                    match w {
                        "as" => Next::Def,
                        "class" => Next::Class,
                        _ => Next::Word,
                    },
                );
            }
            // `= trace p where { … }` — the traced point is a name the family declares
            if w == "trace" && prev == Some(&Tok::Eq) {
                return (Some(Tint::Word), Next::Def);
            }
            // a chain (spec §6.6): the element keyword mid-line, the words standing prefix to
            // it, the joints between links, and `close`.  Each is claimed only in the company a
            // chain puts it in, so a point *named* `tangent` in an argument list stays plain.
            // past the operator's own parentheses, which is where its right operand is:
            // `radius(25) circle base(…)` and `p distance(80) q` are the prefix and the joint
            // they would be without a number on the word.  The same lookahead `chain_starts`
            // reads, so a word this colours as a relation is one the parser settles as one —
            // and computed *here*, in the one arm that reads it, since the loop around this runs
            // per keystroke and every other arm has already returned.
            let j = past_args(toks, i);
            let next_word = word_at(toks, j);
            // both questions off the one cursor: a line ending in a joint word continues its
            // chain onto the next, and `p distance(80)` ends a line as surely as `p equal` does
            let at_line_end = matches!(toks.get(j).map(|(t, _)| t), Some(Tok::Nl) | None);
            if opens_link(w, next_word) {
                // the element keyword names what the link declares; a prefix states a relation
                return match EntKind::parse(w) {
                    Some(_) => (Some(Tint::Word), after_kind(w)),
                    None => (Some(Tint::Relation), Next::Word),
                };
            }
            let at_marker = matches!(toks.get(j).map(|(t, _)| t), Some(Tok::Arrow));
            if (next_word.is_some() || at_line_end || at_marker) && joint_word(w) {
                // `at_marker` is the far-side marker: `A -> equal -> B`
                return (Some(Tint::Relation), Next::Word);
            }
            if w == "close" && at_line_end {
                return (Some(Tint::Word), Next::Word);
            }
            (None, Next::Word)
        }
    }
}

/* -- chains ------------------------------------------------------------------------ */

/// What a chain link stands on.
///
/// A link that *declares* offers the joints beside it a list to read and fill; a link that only
/// *names* an element declared elsewhere offers neither — its boundary is its own declaration's
/// business.  Whether a joint welds the two is **not** read off this distinction: threading is
/// stated at the joint, by the `->` marker, and its absence states the links are separate
/// (spec §6.6) — `a_br equal a_tl` says the two arcs are the same size and nothing whatever
/// about where they meet, and writing `->` beside the word is how one would say more.
enum LinkBody {
    /// `line bottom(b1, b2)` — the chain declares it, so the keyword says what kind it is.
    /// Boxed because a `Decl` is many times a `Ref`, and a chain holds a `Vec` of these.
    Decl(Box<Decl>),
    /// `a_br` — the chain names one declared elsewhere.  What kind it is, only elaboration
    /// knows: a name may be declared further down the file, or come from a component.
    Ref(Ref),
}

/// A joint between two links: whether the `->` marker threads it, the words standing at it (each
/// with whatever stood in its parentheses), and where its text runs — from the marker (or the
/// first word, where there is no marker) through the last word's arguments, and through `close`
/// for the joint that seals a loop.  At least one of marker and word is present; the grammar has
/// no empty joint.
struct Joint {
    thread: bool,
    /// The relations stated at this joint — `-> equal angle(30deg)` states both at the corner
    /// just threaded.  Each word carries its own parentheses and its own span, so each desugars
    /// to a statement of its own.
    words: Vec<(String, Vec<OpArg>, Span)>,
    span: Span,
    /// The joint's own statements are skipped where its structure was refused — a threaded
    /// circle, a `close` over one link — so one mistake is one message.
    sound: bool,
}

/// One link of a chain while it is being read: the unary constraint words standing before it,
/// what it stands on, and where its text sits.
struct Link {
    /// The words standing before it, each with its own parentheses: `horizontal`, `radius(25)`.
    prefixes: Vec<(Name, Vec<OpArg>)>,
    body: LinkBody,
    /// Where the link's text runs, which is not the declaration's: it starts at the element
    /// keyword rather than at the name.
    span: Span,
}

impl Link {
    /// The entity this link is about, as a relation written over it would name it.
    fn entity(&self) -> Ref {
        match &self.body {
            LinkBody::Decl(d) => Ref { root: d.name.clone(), path: Vec::new(), span: d.name.span },
            LinkBody::Ref(r) => r.clone(),
        }
    }

    /// What kind of entity it is, where that is known *here* — a declaration says so with its
    /// keyword, and a name does not say until elaboration resolves it.
    fn kind(&self) -> Option<EntKind> {
        match &self.body {
            LinkBody::Decl(d) => Some(d.kind),
            LinkBody::Ref(_) => None,
        }
    }

    /// Where to point a complaint about this link.  An **anonymous** declaration's name span is
    /// empty — it marks where a name *would* go — and a caret with nothing under it says
    /// nothing, so the link's own text stands in: the keyword a reader can see.
    fn span_of_name(&self) -> Span {
        match &self.body {
            LinkBody::Decl(d) if !d.name.span.is_empty() => d.name.span,
            LinkBody::Decl(_) => self.span,
            LinkBody::Ref(r) => r.span,
        }
    }
}

/// The relation a **polymorphic** word states between two kinds.
///
/// Beside `joint_relation`'s tangency table because it is the same sort of word: drafting
/// vocabulary whose meaning is the pair it stands between, which no registry lookup can answer
/// because the answer is *which constraint to state* rather than how to spell one.  `equal` is
/// the second the language has — a length between lines, a radius between circles or arcs, and
/// nothing at all between one of each, since no constraint equates a length to a radius.
pub fn equal_kind(left: EntKind, right: EntKind) -> Option<CKind> {
    match (left, right) {
        (EntKind::Line, EntKind::Line) => Some(CKind::EqualLength),
        (EntKind::Circle | EntKind::Arc, EntKind::Circle | EntKind::Arc) => {
            Some(CKind::EqualRadius)
        }
        _ => None,
    }
}

/// Whether a word may stand *before* its one operand — `horizontal`, `vertical`, `radius`,
/// `distance` (spec §9.1).  Derived from the operator table by asking it, so a word given a
/// prefix reading later joins the grammar with nothing here to edit.
fn prefix_word(w: &str) -> bool {
    [EntKind::Line, EntKind::Circle, EntKind::Arc]
        .iter()
        .any(|&k| crate::constraints::prefix_op(w, k).is_some())
}

/// The identifier at a token position, where there is one.
///
/// A free function over the slice, with `P::word_at` a one-line delegator, because the colouring
/// walks the same tokens without a parser around them and a second copy of one `match` is a
/// second place for it to change.
fn word_at(toks: &[(Tok, Span)], i: usize) -> Option<&str> {
    match toks.get(i).map(|(t, _)| t) {
        Some(Tok::Ident(n)) => Some(n.as_str()),
        _ => None,
    }
}

/// The token index just past the word at `i` and **its parentheses**, if it has any — the cursor
/// `opens_link` and the joint test read from now that an operator carries its number on the word
/// itself.
///
/// `radius(25) circle base(center: c)` opens a chain exactly as `horizontal line l(a, b)` does,
/// and `p distance(80) q` is a joint exactly as `p equal q` is — but in both, the token after the
/// word is `(` and not the word the test is looking for.  Reading only `i + 1` made the
/// parenthesised prefix open no chain, which routed it to `relation()` and had `refr()` swallow
/// the keyword `circle` as an operand — while the *same* form parsed mid-chain, where `link`
/// reads the arguments itself — and left every parenthesised infix operator uncoloured.
///
/// An **index** and not a word, the shape `past_ref` already uses: a caller needs to ask two
/// things at that position — what word is there, and whether the line ends there — and a lookahead
/// that answered only the first left `p distance(80)` at the end of a line reading as neither.
/// A free function over the token slice because `chain_starts` and `highlight` both ask it, and
/// what `opens_link` already says about itself holds here: written twice, the two drift at once.
fn past_args(toks: &[(Tok, Span)], i: usize) -> usize {
    let mut j = i + 1;
    if toks.get(j).map(|(t, _)| t) != Some(&Tok::P('(')) {
        return j;
    }
    let mut depth = 0i32;
    loop {
        match toks.get(j).map(|(t, _)| t) {
            Some(Tok::P('(')) => depth += 1,
            Some(Tok::P(')')) => {
                depth -= 1;
                if depth == 0 {
                    return j + 1;
                }
            }
            // an unclosed list is a syntax error the parser proper reports; this lookahead only
            // has to stop rather than run off the end
            Some(Tok::Nl) | None => return j,
            _ => {}
        }
        j += 1;
    }
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
    if EntKind::parse(w).is_some() {
        return true; // a declaration names itself — or nothing at all, the name being optional
    }
    // the lookahead: it is a pointer test, where `prefix_word` scans the operator table
    let Some(n) = next else { return false };
    (EntKind::parse(n).is_some() || prefix_word(n)) && prefix_word(w)
}

/// Whether a word may be a declaration's **name**.  With the name optional (§6.1, issue #33),
/// the token after the kind keyword decides what the statement says next, so every word that may
/// follow a declaration is reserved: another element keyword, anything that trails one, and `at`,
/// whose retired seed spelling keeps its message.  `line hint(x: 0, y: 0)` seeds an anonymous
/// line, and does not declare one named `hint`.  Asked by the parser (`decl`) and the colouring
/// (`tint_word`) alike — written twice, the two would drift on exactly these words.
fn names_decl(w: &str) -> bool {
    EntKind::parse(w).is_none() && !trails_decl(w) && w != "at"
}

/// Whether a word may stand *after* a declaration — a trailing clause's own word, or a chain's
/// joint.  **The one spelling** of that list: `class_clause` stops on it, the colouring's class
/// arm stops on it, and `names_decl` reserves it.  Written three times, a word added to a
/// declaration's tail lands in one of them and the colouring and the parser part company.
fn trails_decl(w: &str) -> bool {
    TRAILERS.contains(&w) || joint_word(w)
}

/// What follows an element keyword: a name, and the name is optional — except after `curve`,
/// whose form is `curve name = family(…)` and whose name a contact addresses.  `decl()` makes
/// the same exception, so this is the colouring's half of one rule.
fn after_kind(w: &str) -> Next {
    if w == "curve" {
        Next::Def
    } else {
        Next::DeclName
    }
}

/// How a message spells a declaration whose name is optional: the name where the source wrote
/// one, and the bare kind where it did not.  Both the parser's errors and the elaborator's
/// diagnostics ask, so an anonymous declaration is described one way and never by its key.
pub(crate) fn decl_head(kind: EntKind, name: &str) -> String {
    if hidden(name) {
        kind.as_str().to_string()
    } else {
        format!("{} {name}", kind.as_str())
    }
}

/// Whether a name is one the source could never write: the `#a`-keyed name an anonymous
/// declaration resolves by (`#a41`, and `#a41.p2` for a child it mints) or a block prefix the
/// flattener made (`#5.0.p0`).  `#` never survives the tokenizer inside an identifier, so the
/// test cannot claim a written name.  Who asks: what would *write* a name into the source — the
/// printer, which spells an anonymous declaration without one — and the diagnostics, which
/// spell the kind.
///
/// It answers a question about a **string**, and so cannot tell those two cases apart; anything
/// that needs to knows already and is told (`Decl::named`, `program::Say`, issue #39).
pub fn hidden(name: &str) -> bool {
    name.contains('#')
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

/// A reference as written, for a message about it — and for a writeback that has to spell a
/// reference the source never wrote (a chain-minted `l1.p2`).
pub(crate) fn ref_text(r: &Ref) -> String {
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
    /// A word `decl()` declined as a name because the language reserves it — kept until the
    /// statement ends, so a line that then fails to parse can say the likely cause.  A line
    /// that parses (`line tangent arc` is a chain) needs no saying, so this is only read
    /// beside a failure (`chain_or_one`).
    declined: Option<(String, Span)>,
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
    let mut st = P { src, t: lexed.toks, i: 0, errs, declined: None };
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

    /// `hint at REF` — a seed given as a *place* rather than as a pair of numbers, which is what
    /// a seed inside a trace block is (spec §6.5.1): `hint at t`, `hint at c bearing (u + phase)`.
    ///
    /// `hint(x:, y:)` is the coordinate spelling of the same thing and both lower to the same
    /// tapes, which is why the word is shared.  `hint at (0, 0)` is *not* accepted: coordinates
    /// are keyed now, and one spelling of a thing is the whole point of the clause.
    fn eat_hint_at(&mut self) -> bool {
        if self.peek_word("hint")
            && self.word_at(self.i + 1) == Some("at")
            && self.t.get(self.i + 2).map(|(t, _)| t) != Some(&Tok::P('('))
        {
            self.i += 2;
            return true;
        }
        false
    }

    /// `hint(` — the clause every seed is written in.  The brackets after a name say what the
    /// thing is *made of*; this says where the solve *begins* (spec §6.4).  A number inside one
    /// is a seed and every other number is not, which is what makes "may a solve write this?" a
    /// lexical test rather than an analysis.
    ///
    /// It gives back where the `hint` word stands, since every caller wants the span of the whole
    /// clause and only the one that ate the word knows where it began.
    fn eat_hint_clause(&mut self) -> Option<usize> {
        if self.peek_word("hint") && self.t.get(self.i + 1).map(|(t, _)| t) == Some(&Tok::P('(')) {
            let lo = self.t[self.i].1.lo as usize;
            self.i += 2;
            return Some(lo);
        }
        None
    }

    /// A `hint(…)` standing in a child slot, the opening paren already eaten.
    ///
    /// The same clause as everywhere else, so it is read by the same `hint_body`; what the keys
    /// mean is this table — an anonymous child is a point, and a point has x and y.
    fn kid_seed(&mut self, lo: usize) -> Option<KidSeed> {
        let mut k = KidSeed::default();
        for h in self.hint_body("x: 0, y: 0")? {
            let i = match h.key.as_str() {
                "x" => 0,
                "y" => 1,
                _ => {
                    let m = format!("an anonymous point has no scalar `{}`; it has x and y", h.key);
                    self.fail_at(h.at, &m);
                    return None;
                }
            };
            k.v[i] = h.value.unwrap_or(0.0);
            k.text[i] = (h.value.is_none()).then_some(h.text);
            k.spans[i] = h.span;
        }
        k.span = Span::new(lo, self.prev_hi());
        Some(k)
    }

    /// `style .construction { dash: 7 4; width: 0.5; color: #888888 }`.
    ///
    /// A `#rrggbb` is not one token — the lexer has no colour literal and does not need one, a
    /// colour being the only thing in the language written with a `#`.  So the value of every
    /// property is taken as the run of tokens up to the `;` or the `}`, and read by the property
    /// that wanted it.
    fn style_rule(&mut self) -> Option<StmtKind> {
        let lo = self.here().lo as usize;
        self.i += 1; // `style`
        if !self.want_p('.') {
            return None;
        }
        let name = self.ident()?;
        if !self.want_p('{') {
            return None;
        }
        let mut style = Style::default();
        let mut props: Vec<String> = Vec::new();
        while !self.eat_p('}') {
            if self.eat_p(';') || self.peek() == Some(&Tok::Nl) {
                self.i += usize::from(self.peek() == Some(&Tok::Nl));
                continue;
            }
            let Some(prop) = self.slot_label() else {
                self.fail("a style rule is `property: value`");
                return None;
            };
            let from = self.here().lo as usize;
            let mut values: Vec<f64> = Vec::new();
            while !matches!(self.peek(), Some(Tok::P(';')) | Some(Tok::P('}')) | Some(Tok::Nl) | None)
            {
                if let Some(Tok::Num(v)) = self.peek() {
                    values.push(*v);
                }
                self.i += 1;
            }
            let text = self.text_from(from).trim().to_string();
            if !style.set(&prop, &values, &text) {
                // an unknown property is not an error, exactly as an unmatched class is not:
                // a sheet says what it knows how to say and the rest has no rule
                continue;
            }
            props.push(prop);
        }
        Some(StmtKind::Style(StyleRule { name, style, props, span: Span::new(lo, self.prev_hi()) }))
    }

    /// `class centerline heavy` — the classes a declaration carries, and where they are written.
    ///
    /// Every bare word after `class` belongs to it, up to the next thing a declaration may say:
    /// another trailing clause, or a chain's joint (`class construction tangent arc …` is one
    /// link, and `tangent` is not a class).  `at` is where the clause *would* go when there is
    /// none, so an edit that adds one has somewhere to put it.
    fn class_clause(&mut self, at: usize) -> (Classes, Span) {
        if !self.peek_word("class") {
            return (Classes::default(), Span::new(at, at));
        }
        let lo = self.here().lo as usize;
        self.i += 1;
        let mut c = Classes::default();
        while let Some(Tok::Ident(w)) = self.peek().cloned() {
            if trails_decl(&w) {
                break;
            }
            c.0.push(w);
            self.i += 1;
        }
        (c, Span::new(lo, self.prev_hi()))
    }

    /// `name:` at the head of a slot — the label and nothing else, consumed.
    ///
    /// Matched through the reference and cloned only in the winning arm, the way `eat_word` is:
    /// this is asked once per argument and once per hint, and most of those asks say no.
    fn slot_label(&mut self) -> Option<String> {
        match (self.peek(), self.t.get(self.i + 1).map(|(t, _)| t)) {
            (Some(Tok::Ident(s)), Some(Tok::P(':'))) => {
                let s = s.clone();
                self.i += 2;
                Some(s)
            }
            _ => None,
        }
    }

    /// The body of a `hint(…)`, from just past the `(` to the `)`.
    ///
    /// The grammar is the clause's own and is the same wherever the clause is written; what a
    /// key *means* is not — a declaration's scalars and a constraint's `Param` slots are
    /// different tables — so the keys come back unresolved, each with the span it was written
    /// at, and the caller looks them up and reports an unknown one where it stands.
    fn hint_body(&mut self, eg: &str) -> Option<Vec<Hint>> {
        let mut out = Vec::new();
        while !self.eat_p(')') {
            let at = self.here();
            let Some(key) = self.slot_label() else {
                self.fail(&format!("a hint names what it seeds: `hint({eg})`"));
                return None;
            };
            let (value, text, span) = self.value_text()?;
            out.push(Hint { key, at, value, text, span });
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        Some(out)
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
        self.fail_at(span, msg);
    }

    /// The same, at a span already read — a key whose value has since been consumed is reported
    /// where it was written and not where the parser has got to.
    fn fail_at(&mut self, span: Span, msg: &str) {
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
            "unit" => {
                self.i += 1;
                Some(StmtKind::Unit(self.ident()?))
            }
            "style" => self.style_rule(),
            // **`ccw` and `cw` keep a call.**  Every other statement is a prefix or an infix
            // operator, and under that rule these would be `a ccw(c) b` — which reorders three
            // points that are symmetric, since the predicate is about the *triangle* and not
            // about a pair with a decoration.  Spec §9.6 keeps the call for exactly that reason.
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
            // the gauges are prefix operators like any other: `ground p1`, `fix c.r`
            g if GAUGES.contains(&g) => {
                self.i += 1;
                let ground = w == "ground";
                let r = self.refr()?;
                self.end_of_stmt();
                Some(StmtKind::Gauge(if ground { Gauge::Ground(r) } else { Gauge::Fix(r) }))
            }
            "branch" => {
                self.i += 1;
                if !self.want_p('(') {
                    return None;
                }
                // the key is written **bare** — `branch(ppp:3|4|5, 1)`.  The language has no
                // string literal (a quote is the inch mark, §3.3), and a branch key is one run
                // of characters with no comma in it, so it is taken as the source text up to
                // the comma exactly as a dimension's is taken up to the end of its line.
                let from = self.here().lo as usize;
                while !matches!(self.peek(), Some(Tok::P(',')) | Some(Tok::Nl) | None) {
                    self.i += 1;
                }
                let key = self.text_from(from).trim().to_string();
                if key.is_empty() {
                    self.fail("a raw branch names the construction it decides");
                    return None;
                }
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
            // `claim vertical(rail)` — a relation stated as expected to add no rank.  The colon
            // guard keeps an instance *named* claim (`claim: Tooth(…)`) an instance.
            "claim" if !matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P(':'))) => {
                self.i += 1;
                let mut r = self.relation()?;
                r.claim = true;
                Some(StmtKind::Relation(r))
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
        self.declined = None;
        let before = self.errs.len();
        let got = if !self.chain_starts() {
            let lo = self.here().lo as usize;
            let kind = self.stmt(next_id)?;
            *next_id += 1;
            out.push(Stmt {
                id: StmtId(*next_id),
                kind,
                span: Span::new(lo, self.prev_hi()),
                chained: Chained::No,
            });
            Some(())
        } else {
            self.chain(next_id, out)
        };
        // The name in a declaration is optional, and the words that may follow one are reserved
        // (§6.1) — so a statement that *named* a declaration with one of them no longer parses,
        // and what went wrong is a reservation the errors above cannot see.  Said only beside a
        // failure: `line tangent arc` is a chain, and needs no remark.
        if self.errs.len() > before {
            if let Some((w, span)) = self.declined.take() {
                self.errs.push(SynErr {
                    span,
                    message: format!(
                        "note: `{w}` cannot be a declaration's name — the words that may follow \
                         a declaration are reserved (spec §6.1)"
                    ),
                });
            }
        }
        got
    }

    /// Whether what stands here opens a declaration — possibly a chain of them.
    fn chain_starts(&self) -> bool {
        let Some(Tok::Ident(w)) = self.peek() else { return false };
        let next = word_at(&self.t, past_args(&self.t, self.i));
        // `a_br equal a_tr` — a name, then a word that relates it to another.  Nothing else in
        // the language has that shape: a statement opening with a bare name is an instance, and
        // that is a name followed by a colon.  `claim parallel(…)` has the shape too — a binary
        // relation's name doubles as an infix joint word — but `claim` qualifies a statement,
        // it never names an element.
        // a word that *opens* a statement is not an operand, however the next word reads:
        // `param radius = 50` is a definition and not `param` related to `radius`
        if !OPENERS.contains(&w.as_str())
            && EntKind::parse(w).is_none()
            && !prefix_word(w)
            && !is_operator(w)
        {
            // the operand may be a dotted name — `l.p1 distance(6) l.p2` — so the word that
            // relates it is looked for past the whole reference, not at the next token.  The
            // retired `to` is still recognised here, so `a to k` reaches the chain loop and its
            // migration message rather than a generic refusal.
            if let Some(j) = self.past_ref(self.i) {
                if matches!(self.t.get(j).map(|(t, _)| t), Some(Tok::Arrow))
                    || self.word_at(j).is_some_and(|w| joint_word(w) || w == "to")
                {
                    return true;
                }
            }
        }
        opens_link(w, next)
    }

    /// The token index just past a reference beginning at `j` — a name, then any run of
    /// `.field` and `[index]` — or `None` where no reference begins there.
    fn past_ref(&self, mut j: usize) -> Option<usize> {
        if !matches!(self.t.get(j).map(|(t, _)| t), Some(Tok::Ident(_))) {
            return None;
        }
        j += 1;
        loop {
            match self.t.get(j).map(|(t, _)| t) {
                Some(Tok::P('.')) => j += 2,
                Some(Tok::P('[')) => {
                    let mut d = 0i32;
                    loop {
                        match self.t.get(j).map(|(t, _)| t) {
                            Some(Tok::P('[')) => d += 1,
                            Some(Tok::P(']')) => {
                                d -= 1;
                                if d == 0 {
                                    j += 1;
                                    break;
                                }
                            }
                            Some(Tok::Nl) | None => return Some(j),
                            _ => {}
                        }
                        j += 1;
                    }
                }
                _ => return Some(j),
            }
        }
    }

    /// The identifier at a token position, where there is one — what `opens_link` reads ahead.
    fn word_at(&self, i: usize) -> Option<&str> {
        word_at(&self.t, i)
    }

    /// `[prefix…] decl (joint [prefix…] decl)* [joint "close"]`.
    fn chain(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        let lo = self.here().lo as usize;
        let mut links = vec![self.link()?];
        let mut joints: Vec<Joint> = Vec::new();
        let mut close: Option<Joint> = None;
        loop {
            let start = self.here().lo as usize;
            // `->` says the two links beside it share a boundary point; a word beside it says
            // what else holds at the corner just threaded.  At least one of the two makes a
            // joint, and neither alone implies the other (spec §6.6).
            let mut thread = self.peek() == Some(&Tok::Arrow);
            // where the joint's own text ends — the last marker or word taken, so a doomed
            // joint's splice does not eat a line break the words stepped over
            let mut hi = self.prev_hi();
            if thread {
                self.i += 1;
                hi = self.prev_hi();
                // a line ending in `->` continues the chain on the next, exactly as a line
                // ending in a joint word does
                self.skip_ends();
            }
            // a run of words, each stating a relation at this joint: `-> tangent equal` is a
            // corner that is tangent there, between two links also equal in length.  A word
            // that opens a link is the next link's own — `-> vertical line right(…)` is a
            // plain corner onto a levelled line, not a `vertical` joint — which is the same
            // order of questions the colouring asks (`tint_word`), so the two cannot disagree
            let mut words: Vec<(String, Vec<OpArg>, Span)> = Vec::new();
            loop {
                let Some(Tok::Ident(w)) = self.peek() else { break };
                if !joint_word(w) || opens_link(w, self.word_at(past_args(&self.t, self.i))) {
                    break;
                }
                let w = w.clone();
                let lo = self.here().lo as usize;
                self.i += 1;
                // an infix operator carries its own parentheses: `p1 distance(80) p2` is a
                // chain of one joint, which is the unification that makes a lone statement
                // and a chain one grammar rather than two
                let args = self.op_args(&w)?;
                words.push((w, args, Span::new(lo, self.prev_hi())));
                hi = self.prev_hi();
                // the marker may stand on either side of the words, or both — `A -> equal -> B`
                // is the one joint `A -> equal B` is — and any marker threads.  Read beside its
                // word, before the line break, so a continuation onto the next line never picks
                // up a marker that was written to start a statement there
                if self.peek() == Some(&Tok::Arrow) {
                    thread = true;
                    self.i += 1;
                    hi = self.prev_hi();
                    break;
                }
                // a line ending in a joint word continues its chain onto the next
                self.skip_ends();
            }
            // the retired 0.8 list, caught so a document written against it says what to write
            if (thread || !words.is_empty()) && self.peek() == Some(&Tok::P('(')) {
                self.fail("a joint states its relations as bare words: `-> equal angle(30deg)` (spec §6.6)");
            }
            if !thread && words.is_empty() {
                // the retired corner word, caught here so a 0.7 document says what to write
                if self.peek_word("to") {
                    self.fail("`to` is retired: a corner is written `->` (spec §6.6)");
                    self.i += 1;
                    thread = true;
                    hi = self.prev_hi();
                } else {
                    break;
                }
            }
            // a line ending in a joint continues the chain on the next — the one place a
            // statement runs past its line's end
            self.skip_ends();
            if self.eat_word("close") {
                if !thread {
                    self.fail("a loop is a thread: a chain closes with `-> close`");
                }
                close = Some(Joint {
                    thread: true,
                    words,
                    span: Span::new(start, self.prev_hi()),
                    sound: true,
                });
                break;
            }
            joints.push(Joint { thread, words, span: Span::new(start, hi), sound: true });
            links.push(self.link()?);
        }
        // the trailing clauses a statement may carry — a lone infix operator is a one-joint
        // chain, so it carries them here as it would anywhere else
        let mut place = None;
        let mut place_span = Span::default();
        let mut seeds: Vec<OpArg> = Vec::new();
        loop {
            if self.eat_hint_clause().is_some() {
                for h in self.hint_body("t: 0.4")? {
                    seeds.push(h.into());
                }
            } else if place.is_none() && !joints.is_empty() && self.peek_word("at") {
                // a placement qualifies a *dimension*, so it is read only where the line states
                // one — after a declaration, `at (…)` is not a clause the language has
                let at = self.here().lo as usize;
                self.i += 1;
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
                place_span = Span::new(at, self.prev_hi());
            } else {
                break;
            }
        }
        self.end_of_stmt();
        let whole = Span::new(lo, self.prev_hi());
        // whether the line's end can take a trailing clause appended later: a chain ending
        // in a name (or sealed by `close`) can, one ending in a declaration cannot — the
        // declaration reads a trailing `at` as its own retired seed spelling
        let open_end = close.is_some()
            || matches!(links.last().map(|l| &l.body), Some(LinkBody::Ref(_)));
        let first = out.len();
        self.desugar(links, joints, close, whole, next_id, out);
        // a placement qualifies exactly one dimension (§13.1), so it is attached to the one
        // relation the line states, wherever that fell among the links.  A line stating
        // several offers no way to say which — guessing the first put callouts on statements
        // nobody measured — so both none and several are refused.  Where no placement was
        // written and the line offers a spot, the spot one *would* take is recorded all the
        // same — an empty span at the insertion point, `Decl::hint_span`'s device — so
        // `reconcile` can write a dragged callout down without re-deriving the line.
        {
            let mut rels = out[first..]
                .iter_mut()
                .filter(|s| matches!(s.kind, StmtKind::Relation(_)));
            match (rels.next(), rels.next()) {
                (Some(one), None) => {
                    if let StmtKind::Relation(r) = &mut one.kind {
                        r.place = place;
                        r.place_span = match (place, open_end) {
                            (Some(_), _) => place_span,
                            (None, true) => Span::new(whole.hi as usize, whole.hi as usize),
                            (None, false) => Span::default(),
                        };
                    }
                }
                (Some(_), Some(_)) if place.is_some() => self.errs.push(SynErr {
                    span: place_span,
                    message: "a placement qualifies one dimension, and this line states \
                              several relations (§13.1)"
                        .to_string(),
                }),
                (None, _) if place.is_some() => self.errs.push(SynErr {
                    span: place_span,
                    message: "a placement qualifies a dimension, and this line states no \
                              relation (§13.1)"
                        .to_string(),
                }),
                _ => {}
            }
        }
        // a seed qualifies the one statement the line states, which for a lone infix operator
        // is the statement itself
        if let Some(StmtKind::Relation(r)) = out.get_mut(first).map(|s| &mut s.kind) {
            if let Some(w) = r.poly.as_mut() {
                w.args.extend(seeds);
            }
        }
        Some(())
    }

    /// `[prefix…] KIND name(…)`, or a bare name — the two things a link may stand on.
    fn link(&mut self) -> Option<Link> {
        let mut prefixes: Vec<(Name, Vec<OpArg>)> = Vec::new();
        let kind = loop {
            let Some(Tok::Ident(w)) = self.peek().cloned() else {
                self.fail("expected an element");
                return None;
            };
            if let Some(k) = EntKind::parse(&w) {
                break Some(k);
            }
            // a name, not a keyword: the link stands on something declared elsewhere.  A prefix
            // word only reaches here when an element follows it, so this cannot swallow one.
            if prefixes.is_empty() && !prefix_word(&w) {
                break None;
            }
            if !prefix_word(&w) {
                self.fail("expected an element");
                return None;
            }
            // a prefix word carries its own parentheses like any other operator:
            // `radius(25) circle base(center: c)`
            let name = Name { text: w, span: self.here() };
            self.i += 1;
            let args = self.op_args(&name.text)?;
            prefixes.push((name, args));
        };
        let lo = self.here().lo as usize;
        let Some(kind) = kind else {
            let r = self.refr()?;
            return Some(Link {
                prefixes,
                span: r.span,
                body: LinkBody::Ref(r),
            });
        };
        self.i += 1; // the kind keyword
        let decl = self.decl(kind)?;
        let body = LinkBody::Decl(Box::new(decl));
        Some(Link { prefixes, body, span: Span::new(lo, self.prev_hi()) })
    }

    /// The statements a chain is sugar for, in the order their text sits: each link's prefix
    /// relations, its declaration, and between links the joint that binds them.
    fn desugar(
        &mut self,
        mut links: Vec<Link>,
        mut joints: Vec<Joint>,
        mut close: Option<Joint>,
        whole: Span,
        next_id: &mut u32,
        out: &mut Vec<Stmt>,
    ) {
        let chained = links.len() > 1 || close.is_some();
        let n = links.len();
        // **A lone infix operator is a one-joint chain**, and that is a unification rather than
        // a new case — but the statement it makes occupies its whole line, so it is recorded as
        // the ordinary statement it is: a whole-line span to splice, and no chain to be part of.
        let lone = n == 2
            && joints.len() == 1
            && close.is_none()
            && !joints[0].thread
            && joints[0].words.len() == 1
            && links.iter().all(|l| l.kind().is_none());

        // **threading is stated at the joint, never inferred** (spec §6.6): `->` says the two
        // links beside it share a boundary point, and its absence says they do not — so a chain
        // may mix declarations and names freely, and each marker is judged where it stands.
        // A marker needs an end on each side it can see, which is exactly a line or an arc; a
        // side that only names an element has a kind only elaboration knows, and is trusted to
        // the point the other side names.
        if close.is_some() && n < 2 {
            self.errs.push(SynErr {
                span: links[0].span_of_name(),
                message: "a chain closes over at least two elements".to_string(),
            });
            if let Some(c) = &mut close {
                c.sound = false;
            }
        }
        let mut endless = vec![false; n]; // reported once per link, however many markers reach it
        for k in 0..joints.len() + usize::from(close.as_ref().is_some_and(|c| c.sound)) {
            let (thread, li, ri) = match joints.get(k) {
                Some(j) => (j.thread, k, k + 1),
                None => (true, n - 1, 0),
            };
            if !thread {
                continue;
            }
            let mut sound = true;
            for side in [li, ri] {
                if links[side].kind().is_some_and(|k| k.ends().is_none()) {
                    if !endless[side] {
                        self.errs.push(SynErr {
                            span: links[side].span_of_name(),
                            message: format!(
                                "a corner joins lines and arcs; a {} has no ends to thread",
                                links[side].kind().map(|k| k.as_str()).unwrap_or("thing")
                            ),
                        });
                        endless[side] = true;
                    }
                    sound = false;
                }
            }
            if !sound {
                match joints.get_mut(k) {
                    Some(j) => j.sound = false,
                    None => close.as_mut().expect("the close joint").sound = false,
                }
            }
        }

        // threading: at each threaded joint the shared point is named by exactly one side, by
        // both in agreement, or — between two declarations — by nobody, in which case the chain
        // mints it (`thread`).  An end no marker reaches is an implicit child like any other
        // unwritten slot (§6.2): `line l1 -> line l2` is two lines and three points, one shared.
        for k in 0..joints.len() {
            if joints[k].thread && joints[k].sound {
                self.thread(&mut links, k, k + 1, joints[k].span);
            }
        }
        if close.as_ref().is_some_and(|c| c.sound) {
            let sp = close.as_ref().expect("just checked").span;
            self.thread(&mut links, n - 1, 0, sp);
        }
        // the links are consumed below, so what a joint needs of them — the entity and its kind
        // where that is known — is taken first, and only where there are joints to need it
        let sig: Vec<(Ref, Option<EntKind>)> = match chained {
            true => links.iter().map(|l| (l.entity(), l.kind())).collect(),
            false => Vec::new(),
        };
        // how each worded joint splices when its statement is doomed (`Chained`): a threaded
        // one steps down to the bare corner; an unthreaded one becomes a statement break, its
        // span grown over a terminal name-link that a break would leave dangling; in a chain
        // that closes no break is safe — it would re-aim the `close` — so the joint is Stuck
        let spell: Vec<(Span, Fall)> = joints
            .iter()
            .enumerate()
            .map(|(k, j)| {
                if j.thread {
                    return (j.span, Fall::Joint);
                }
                if close.is_some() {
                    return (j.span, Fall::Stuck);
                }
                let mut sp = j.span;
                if k == 0 && links[0].kind().is_none() {
                    sp = Span::new(links[0].span.lo as usize, sp.hi as usize);
                }
                if k + 1 == n - 1 && links[n - 1].kind().is_none() {
                    // …and through the trailing clauses: a placement or a seed after the
                    // chain qualifies this line's statements, so text a break left standing
                    // behind the taken name-link would dangle
                    sp = Span::new(sp.lo as usize, whole.hi as usize);
                }
                (sp, Fall::Infix)
            })
            .collect();
        let at = |i: usize| (&sig[i].0, sig[i].1);
        for (i, link) in links.into_iter().enumerate() {
            if i > 0 && joints[i - 1].sound {
                let j = &joints[i - 1];
                for k in 0..j.words.len() {
                    let (w, args, wspan) = &j.words[k];
                    // a word with siblings spans itself alone — the blanks around it are the
                    // splice's business, so a comment or a line break between two words is
                    // never part of either — and carries the joint's one-word doom for when
                    // the whole joint falls; the joint's only word steps down to the marker
                    // or to a statement break (`spell`)
                    let (span, how) = if lone {
                        (whole, Chained::No)
                    } else if j.words.len() > 1 {
                        let (of, fall) = spell[i - 1];
                        let out_of = j.words.len() as u32;
                        (*wspan, Chained::Member { of, fall, out_of })
                    } else {
                        let (sp, fall) = spell[i - 1];
                        (sp, fall.into())
                    };
                    out.extend(
                        self.joint_stmt(w, args, *wspan, span, at(i - 1), at(i), j.thread, how, next_id),
                    );
                }
            }
            let ent = match &link.body {
                LinkBody::Decl(d) => Ref { root: d.name.clone(), path: Vec::new(), span: d.name.span },
                LinkBody::Ref(r) => r.clone(),
            };
            for (word, args) in &link.prefixes {
                let sp = word.span;
                let rel = Relation {
                    kind: CKind::Coincident,   // a placeholder `program::constrain` replaces
                    args: Vec::new(),
                    place: None,
                    place_span: Span::default(),
                    poly: Some(Written {
                        word: word.clone(),
                        fixity: Fixity::Prefix,
                        ops: vec![ent.clone()],
                        args: args.clone(),
                        span: sp,
                    }),
                    claim: false,
                };
                *next_id += 1;
                out.push(Stmt {
                    id: StmtId(*next_id),
                    kind: StmtKind::Relation(rel),
                    span: sp,
                    chained: Chained::Prefix,
                });
            }
            // a link that only *names* an element declares nothing, so it emits no statement of
            // its own — the whole of what it contributes is being one end of its joints
            let LinkBody::Decl(decl) = link.body else { continue };
            let decl = *decl;
            *next_id += 1;
            out.push(Stmt {
                id: StmtId(*next_id),
                kind: StmtKind::Decl(decl),
                span: link.span,
                chained: if chained { Chained::Link } else { Chained::No },
            });
        }
        if let Some(c) = close {
            let mut sealed = Vec::new();
            if c.sound {
                for k in 0..c.words.len() {
                    let (w, args, wspan) = &c.words[k];
                    let (span, how) = if c.words.len() > 1 {
                        let out_of = c.words.len() as u32;
                        (*wspan, Chained::Member { of: c.span, fall: Fall::Close, out_of })
                    } else {
                        (c.span, Chained::Close)
                    };
                    sealed.extend(
                        self.joint_stmt(w, args, *wspan, span, at(n - 1), at(0), true, how, next_id),
                    );
                }
            }
            if sealed.is_empty() {
                // `-> close` states nothing, so no statement owns its words; the last link's
                // span grows over them, or an append would land in the middle of the chain
                if let Some(last) = out.last_mut() {
                    last.span = Span::new(last.span.lo as usize, c.span.hi as usize);
                }
            } else {
                out.extend(sealed);
            }
        }
    }

    /// The statement one joint states, where it states one — a plain corner states nothing.
    #[allow(clippy::too_many_arguments)]
    fn joint_stmt(
        &mut self,
        word: &str,
        args: &[OpArg],
        at: Span,
        span: Span,
        left: (&Ref, Option<EntKind>),
        right: (&Ref, Option<EntKind>),
        threaded: bool,
        chained: Chained,
        next_id: &mut u32,
    ) -> Option<Stmt> {
        let rel = self.joint_relation(word, args, at, left, right, threaded)?;
        *next_id += 1;
        Some(Stmt { id: StmtId(*next_id), kind: StmtKind::Relation(rel), span, chained })
    }

    /// Resolve one threaded joint's shared point between link `li` (its exit) and link `ri`
    /// (its entry).
    fn thread(&mut self, links: &mut [Link], li: usize, ri: usize, at: Span) {
        fn decl_of(l: &Link) -> Option<&Decl> {
            match &l.body {
                LinkBody::Decl(d) => Some(d),
                LinkBody::Ref(_) => None,
            }
        }
        // which slot each side threads through, where that side is declared here.  A link that
        // only *names* an element has no list to read or fill — its boundary is its own
        // declaration's business — so the declared side must say where the two meet, usually
        // by the existing element's own child (`line l(a, k.start) -> tangent k`).
        let exit = links[li].kind().and_then(|k| k.ends()).map(|(_, ex)| ex);
        let entry = links[ri].kind().and_then(|k| k.ends()).map(|(en, _)| en);
        // a joint threads a *name*: it welds two links to one point, and only a name says
        // which.  A slot seeded with `hint(…)` names nothing, so it reads as unfilled here and
        // the other side must say where they meet.
        let slot = |i: usize, k: Option<usize>| {
            k.and_then(|k| {
                decl_of(&links[i])
                    .and_then(|d| d.children.get(k))
                    .and_then(|v| v.first())
                    .and_then(|kid| kid.as_ref())
                    .cloned()
            })
        };
        let (left, right) = (slot(li, exit), slot(ri, entry));
        // Write a name into a declared side's boundary slot.  A side that only names an element
        // has no list to fill; and a name that already denotes exactly that slot — the link's
        // own dotted boundary, which a written-back chain uses to name the shared point — is
        // left alone rather than written over itself, which would be a reference with no floor.
        fn fill(link: &mut Link, slot: Option<usize>, r: Ref) {
            let Some(k) = slot else { return };
            if let (Some(kind), LinkBody::Decl(d)) = (link.kind(), &mut link.body) {
                if let [Seg::Field(f)] = r.path.as_slice() {
                    if r.root.text == d.name.text && f.text == boundary_name(kind, k) {
                        return;
                    }
                }
                d.children[k] = vec![Kid::Ref(r)];
            }
        }
        // Whether link `a` is built before link `b`: phase 2 builds per kind in declaration
        // order of `EntKind` (`primitives()` order), and within a kind in statement order,
        // which for a chain is link order.
        fn builds_first(a: &Link, ia: usize, b: &Link, ib: usize) -> bool {
            let ord = |l: &Link| l.kind().map(|k| k as usize).unwrap_or(usize::MAX);
            (ord(a), ia) < (ord(b), ib)
        }
        match (left, right) {
            (Some(l), Some(r)) => {
                if !refs_eq(&l, &r) {
                    self.errs.push(SynErr {
                        span: at,
                        message: format!(
                            "the joint names two points: `{}` leaves at `{}` and `{}` arrives \
                             at `{}`",
                            ref_text(&links[li].entity()),
                            ref_text(&l),
                            ref_text(&links[ri].entity()),
                            ref_text(&r),
                        ),
                    });
                }
            }
            (Some(l), None) => {
                fill(&mut links[ri], entry, l);
            }
            (None, Some(r)) => {
                fill(&mut links[li], exit, r);
            }
            (None, None) => {
                // between two declarations the chain mints the point itself: the boundary of
                // the side built first is an anonymous child with a name — the dotted path
                // *is* the name (§6.2) — so the other side's slot is filled with exactly that
                // name.  The side built later takes the fill, so the name exists by the time
                // it resolves; a side that only names an element has no kind to read a
                // boundary field off, so there the point must be named where it stands.
                let lf = links[li].kind().zip(exit).map(|(k, s)| boundary_name(k, s));
                let rf = links[ri].kind().zip(entry).map(|(k, s)| boundary_name(k, s));
                let dotted = |root: Name, f: &str| Ref {
                    root,
                    path: vec![Seg::Field(Name::new(f))],
                    span: Span::default(),
                };
                match (lf, rf) {
                    (Some(lf), Some(rf)) => {
                        if builds_first(&links[li], li, &links[ri], ri) {
                            let r = dotted(links[li].entity().root, lf);
                            fill(&mut links[ri], entry, r);
                        } else {
                            let r = dotted(links[ri].entity().root, rf);
                            fill(&mut links[li], exit, r);
                        }
                    }
                    _ => self.errs.push(SynErr {
                        span: at,
                        message: format!(
                            "neither `{}` nor `{}` names the point where they meet",
                            ref_text(&links[li].entity()),
                            ref_text(&links[ri].entity())
                        ),
                    }),
                }
            }
        }
    }

    /// The relation one worded joint states — and, where the joint threads, an error for a pair
    /// the vocabulary has no regular form for, which is refused rather than stated as a bare
    /// tangency over a shared point (a double root no rank tolerance can read).
    ///
    /// A declared link's kind is known here; a named one's is elaboration's to say — so a word
    /// that needs kinds it does not have travels instead, and `program::constrain` settles it
    /// once the entities are resolved.  What the marker adds is the one thing the operator
    /// cannot know: *which end* two links meet at, read off the direction of travel.
    fn joint_relation(
        &mut self,
        word: &str,
        extra: &[OpArg],
        at: Span,
        left: (&Ref, Option<EntKind>),
        right: (&Ref, Option<EntKind>),
        threaded: bool,
    ) -> Option<Relation> {
        use EntKind::{Arc, Line};
        let (lref, lk) = left;
        let (rref, rk) = right;
        // A joint is the infix operator its word already is, written between two links instead
        // of between two names — so it makes the same `Written` a lone statement does, and
        // `program::constrain` settles both.  The chain contributes the one thing it knows and
        // the operator cannot: *which end* two links meet at.
        let written = |w: &str, ops: Vec<Ref>, args: Vec<OpArg>| {
            Some(Relation {
                kind: CKind::Coincident,
                args: Vec::new(),
                place: None,
                place_span: Span::default(),
                poly: Some(Written {
                    word: Name { text: w.to_string(), span: at },
                    fixity: Fixity::Infix,
                    ops,
                    args,
                    span: at,
                }),
                claim: false,
            })
        };
        let end = |w: &str| {
            vec![OpArg::Named(Name { text: "at".into(), span: at }, Arg::Word(w.to_string()))]
        };
        // no marker, no corner: the word is the ordinary infix operator between the two, as it
        // is between two names — for `tangent`, the well-conditioned bare pair, which is the
        // correct statement exactly when the two are separate
        if !threaded {
            return written(word, vec![lref.clone(), rref.clone()], extra.to_vec());
        }
        match (lk, rk) {
            // the joint knows the shared point, so tangency is stated *at* it — the regular
            // form, with `at:` read off the direction of travel
            (Some(Line), Some(Arc)) if word == "tangent" => {
                written("tangent", vec![rref.clone(), lref.clone()], end("start"))
            }
            (Some(Arc), Some(Line)) if word == "tangent" => {
                written("tangent", vec![lref.clone(), rref.clone()], end("end"))
            }
            // a corner between a fresh element and one declared elsewhere: the declared side
            // says which of its ends was threaded, and elaboration settles the pair once the
            // name's kind is known — the `at:` selector is what keeps the statement the regular
            // form there too, never the bare pair over a coincidence
            (Some(Arc), None) if word == "tangent" => {
                written("tangent", vec![lref.clone(), rref.clone()], end("end"))
            }
            (None, Some(Arc)) if word == "tangent" => {
                written("tangent", vec![rref.clone(), lref.clone()], end("start"))
            }
            (Some(Line), None) if word == "tangent" => {
                written("tangent", vec![lref.clone(), rref.clone()], end("p2"))
            }
            // the named side can only sensibly be an arc here (a line meeting a line tangent is
            // collinear, and needs both declared to say so); `at: end` is the arc's exit, and a
            // name of any other kind is refused where its kind becomes known
            (None, Some(Line)) if word == "tangent" => {
                written("tangent", vec![lref.clone(), rref.clone()], end("end"))
            }
            // two straight runs meeting tangent share a point and a direction: collinear
            (Some(Line), Some(Line)) if word == "tangent" => {
                written("parallel", vec![lref.clone(), rref.clone()], extra.to_vec())
            }
            // two arcs meeting at a corner already touch there, so `TangentCircleCircle` would
            // be a row that is zero at every solution — a *tangency between names* is a real
            // statement, but at a shared corner there is nothing left for it to say
            (Some(a), Some(b))
                if word == "tangent"
                    && matches!(a, Arc | EntKind::Circle)
                    && matches!(b, Arc | EntKind::Circle) =>
            {
                self.errs.push(SynErr {
                    span: at,
                    message: format!(
                        "`tangent` does not join a {} to a {} at a corner: they already meet \
                         there, and there is no regular form left to state",
                        a.as_str(),
                        b.as_str()
                    ),
                });
                None
            }
            _ => written(word, vec![lref.clone(), rref.clone()], extra.to_vec()),
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
        // **The name is optional**, independently of everything after it (issue #33): `line`
        // alone, `line(p1, p2)` and `line hint(x: 0, y: 0)` are all anonymous forms.  The token
        // after the kind keyword decides — an identifier that may be a name is one, and a word
        // reserved for what can follow a declaration (`names_decl`) is read as itself.  An
        // anonymous declaration still needs a key the desugared statements can resolve by — a
        // chain's corner welds by *name* — so it is given one the tokenizer can never produce,
        // `#a` and its own offset (the flattener's block-prefix device, marked apart); its
        // span is empty at the point a real name would go, which is where `edit::reconcile`
        // splices one the moment a statement must say it.  `curve` keeps requiring a name: its
        // form is `curve name = family(…)`, and the name is what the contacts address.
        // Curve first, and on its own: its name is required, so it reaches `ident()` — and that
        // error — whatever stands next, identifier or not.
        let named = kind == EntKind::Curve
            || matches!(self.peek(), Some(Tok::Ident(w)) if names_decl(w));
        let name = if named {
            self.ident()?
        } else {
            // an Ident declined here was very possibly *meant* as a name — remembered, so a
            // line that then fails to parse can say so (`chain_or_one`)
            if let (Some(Tok::Ident(w)), None) = (self.peek(), &self.declined) {
                self.declined = Some((w.clone(), self.here()));
            }
            let at = self.prev_hi();
            Name { text: format!("#a{at}"), span: Span::new(at, at) }
        };
        // how an error spells this statement's head — computed at the failure, since every
        // declaration that parses would otherwise allocate a string nothing reads
        let head = || decl_head(kind, &name.text);
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
            let mut args: Vec<Vec<Kid>> = vec![Vec::new()];
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
                        None => args[0].push(Kid::Ref(self.refr()?)),
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
            let at = self.prev_hi();
            let (class, class_span) = self.class_clause(at);
            return Some(Decl {
                kind,
                name,
                named,
                children: args,
                seed: Vec::new(),
                seed_text: Vec::new(),
                seed_spans: Vec::new(),
                hint_span: None,
                knots: None,
                def,
                values,
                domain,
                class,
                class_span,
                seed_at: None,
            });
        }
        let mut children: Vec<Vec<Kid>> = Vec::new();
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
                let label = self.slot_label();
                match label {
                    // the brackets after the name are *what the thing is made of*; where the
                    // solve begins is the `hint(…)` after them (spec §6.4)
                    Some(l) if scalars.contains(&l.as_str()) => {
                        let head = head();
                        self.fail(&format!(
                            "`{l}` is a seed, and a seed goes in a `hint(…)` clause: \
                             `{head}(…) hint({l}: …)`"
                        ));
                        return None;
                    }
                    _ => {
                        // a slot carries a name or a seed, and nothing else says "anonymous":
                        // an entity whose children are all unseeded writes no list at all
                        let kid = match self.eat_hint_clause() {
                            Some(lo) => Kid::Hint(self.kid_seed(lo)?),
                            None => Kid::Ref(self.refr()?),
                        };
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
                            g.push(kid);
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
        // trailing clauses, in any order: `hint(x: 0, y: 0)` or `hint at REF [bearing (…)]`,
        // `knots [...]`, `class …`.  Where a clause *would* go if it is not written is the
        // point we are standing on now, before any of them: that is what writeback appends at.
        let mut knots = None;
        let mut class = Classes::default();
        let mut class_span = Span::default();
        let mut seed_at = None;
        let insert = self.prev_hi();
        let mut hint_span = Span::new(insert, insert);
        loop {
            if self.eat_hint_at() {
                // a place named geometrically: `hint at t`, `hint at c bearing (u + phase)`
                let what = self.refr()?;
                let bearing =
                    if self.eat_word("bearing") { Some(self.paren_expr()?) } else { None };
                seed_at = Some(AtRef { what, bearing });
            } else if let Some(lo) = self.eat_hint_clause() {
                // `hint(x: 0, y: 12)` — keyed, keys in any order, an omitted scalar is 0
                for h in self.hint_body("x: 0, y: 0")? {
                    let Some(i) = scalars.iter().position(|&s| s == h.key) else {
                        let m = format!("`{}` has no scalar `{}` to seed", kind.as_str(), h.key);
                        self.fail_at(h.at, &m);
                        return None;
                    };
                    seed[i] = h.value.unwrap_or(0.0);
                    seed_text[i] = (h.value.is_none()).then_some(h.text);
                    seed_spans[i] = h.span;
                }
                hint_span = Span::new(lo, self.prev_hi());
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
            } else if self.peek_word("class") {
                let (c, sp) = self.class_clause(insert);
                if c.is_empty() {
                    self.fail("`class` names at least one class");
                    return None;
                }
                class = c;
                class_span = sp;
            } else if self.peek_word("hint") || self.peek_word("at") {
                // the retired coordinate spellings, `hint at (0, 0)` and a bare `at (0, 0)` —
                // what every document in the library said until the clause arrived, so the
                // reader most likely to meet an error here is the one holding one of them
                let head = head();
                self.fail(&format!("a coordinate seed is keyed now: `{head} hint(x: …, y: …)`"));
                return None;
            } else {
                break;
            }
        }
        Some(Decl {
            kind,
            name,
            named,
            children,
            seed,
            seed_text,
            seed_spans,
            hint_span: Some(hint_span),
            knots,
            def,
            values,
            domain,
            class,
            class_span: if class_span.is_empty() { Span::new(insert, insert) } else { class_span },
            seed_at,
        })
    }

    /// One constraint, written as an operator (spec §9.1).
    ///
    /// `PREFIX [( args )] OPERAND` or `OPERAND INFIX [( args )] OPERAND`, then the trailing
    /// clauses every statement may carry: `hint(t: 0.4)` and the callout's `at (t, r)`.
    ///
    /// Nothing is *resolved* here.  What a word means depends on the kinds of its operands, and
    /// a name does not carry its kind until elaboration — so the word and its parentheses travel
    /// in `Relation::poly` and `program::constrain` settles them.  One path, where 0.6 had a
    /// longhand and a chain.
    fn relation(&mut self) -> Option<Relation> {
        let lo = self.here().lo as usize;
        let (word, fixity, ops, args) = match self.peek().cloned() {
            Some(Tok::Ident(w)) if is_operator(&w) => {
                let word = Name { text: w, span: self.here() };
                self.i += 1;
                let args = self.op_args(&word.text)?;
                let r = self.refr()?;
                (word, Fixity::Prefix, vec![r], args)
            }
            _ => {
                let left = self.refr()?;
                let Some(Tok::Ident(w)) = self.peek().cloned() else {
                    self.fail("a statement relates two things with a word between them");
                    return None;
                };
                if !is_operator(&w) {
                    self.fail(&format!("`{w}` is not a constraint"));
                    return None;
                }
                let word = Name { text: w, span: self.here() };
                self.i += 1;
                let args = self.op_args(&word.text)?;
                let right = self.refr()?;
                (word, Fixity::Infix, vec![left, right], args)
            }
        };
        self.relation_tail(word, fixity, ops, args, lo)
    }

    /// Everything a relation statement may carry after its operands.
    fn relation_tail(
        &mut self,
        word: Name,
        fixity: Fixity,
        ops: Vec<Ref>,
        args: Vec<OpArg>,
        lo: usize,
    ) -> Option<Relation> {
        let mut args = args;
        let mut place = None;
        let mut place_span = Span::default();
        loop {
            if self.eat_hint_clause().is_some() {
                // a seed for a slot the constraint owns — the same clause as everywhere else,
                // and read by the same body, so one hint is parsed in one place
                for h in self.hint_body("t: 0.4")? {
                    args.push(h.into());
                }
            } else if place.is_none() && self.peek_word("at") {
                let at = self.here().lo as usize;
                self.i += 1;
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
                place_span = Span::new(at, self.prev_hi());
            } else {
                break;
            }
        }
        self.end_of_stmt();
        // where a placement would go when none was written — an empty span at the insertion
        // point, `Decl::hint_span`'s device, spliced by `reconcile` when a callout is dragged
        if place.is_none() {
            let at = self.prev_hi();
            place_span = Span::new(at, at);
        }
        Some(Relation {
            kind: CKind::Coincident,   // a placeholder `program::constrain` replaces
            args: Vec::new(),
            place,
            place_span,
            poly: Some(Written {
                word,
                fixity,
                ops,
                args,
                span: Span::new(lo, self.prev_hi()),
            }),
            claim: false,
        })
    }

    /// What stood in an operator's parentheses — nothing at all where there are none.
    ///
    /// An **unlabelled** item is the number, except for the three words that take a third entity
    /// (`symmetry`, `ccw`, `cw`): the word decides, which is why this takes it.  The number is
    /// read as raw text and handed to `expr.rs`, exactly as the text after `==` was — the
    /// dimension sub-language is that module's, and a second tokenizer here would be a second
    /// copy of rules like the one that makes `3 1/8` a number and `1' 6"` one length.
    fn op_args(&mut self, word: &str) -> Option<Vec<OpArg>> {
        if !self.eat_p('(') {
            return Some(Vec::new());
        }
        let takes_entity = matches!(word, "symmetry" | "ccw" | "cw");
        let mut out = Vec::new();
        while !self.eat_p(')') {
            match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t.clone())) {
                (Some(Tok::Ident(n)), Some(Tok::P(':'))) => {
                    let name = Name { text: n, span: self.here() };
                    self.i += 2;
                    let v = self.sel_value()?;
                    out.push(OpArg::Named(name, v));
                }
                // `t == 0.4` — the same slot the `hint(…)` clause seeds, *pinned*.  The whole
                // of the value is kept, expression and all: written inside a component a pin
                // reads the parameters in scope (`t == t0`), which `flatten` settles later, and
                // taking only `value` here would pin every one of them at 0.
                (Some(Tok::Ident(key)), Some(Tok::EqEq)) => {
                    let at = self.here();
                    self.i += 2;
                    let (value, text, span) = self.value_text()?;
                    let arg = seed_arg(value, text, span, true);
                    out.push(OpArg::Slot { key: Name { text: key, span: at }, arg });
                }
                _ if takes_entity => out.push(OpArg::Ent(self.refr()?)),
                _ => {
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
                        self.fail("expected the number this states");
                        return None;
                    }
                    let hi = self.prev_hi();
                    out.push(OpArg::Dim(text, Span::new(from, hi)));
                }
            }
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        Some(out)
    }

    /// A selector's value: `side: -1`, `at: start`, `external: true`, `along: x`.
    fn sel_value(&mut self) -> Option<Arg> {
        match self.peek().cloned() {
            Some(Tok::Num(_)) | Some(Tok::P('-')) | Some(Tok::P('+')) => {
                Some(Arg::Num(self.number()?))
            }
            Some(Tok::Ident(w)) if w == "true" || w == "false" => {
                self.i += 1;
                Some(Arg::Bool(w == "true"))
            }
            Some(Tok::Ident(w)) => {
                self.i += 1;
                Some(Arg::Word(w))
            }
            _ => {
                self.fail("expected a value");
                None
            }
        }
    }

    /// Everything after `==` to the end of the logical line, as written.
    ///
    /// Not tokenized: the dimension sub-language is `expr.rs`'s, and lexing it a second time here
    /// would be a second copy of rules like the one that makes `3 1/8` a number and `31/2` a
    /// division.  A trailing ` at (u, v)` is a placement rather than part of the expression —
    /// unambiguous, because a call in that language is `name(` with no space before the paren.
    fn raw_dimension(
        &mut self,
        from: usize,
    ) -> (String, Span, Option<((f64, f64), Span)>, usize) {
        let bytes = self.src.as_bytes();
        let mut end = from;
        while end < bytes.len() && bytes[end] != b'\n' && bytes[end] != b';' {
            if bytes[end] == b'/' && matches!(bytes.get(end + 1), Some(b'/') | Some(b'*')) {
                break;
            }
            end += 1;
        }
        let mut text = &self.src[from..end];
        // a trailing `hint(…)` ends the dimension and starts the statement's trailing clauses;
        // stopping *here* hands those tokens back to the loop that reads them, rather than
        // teaching this function a second grammar.  (No kind carries both a dimension and a
        // `Param` slot today, so this is a guard rather than a path anything walks.)
        if let Some(h) = text.find(" hint(") {
            end = from + h;
            text = &text[..h];
        }
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
                        // the span of `at (...)` itself, leading space excluded: a replacement
                        // keeps the space, and a removal takes it with `back_over_spaces`
                        let lo = from + at + 1;
                        let hi = from + at + 5 + close + 1;
                        place = Some(((nums[0], nums[1]), Span::new(lo, hi)));
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

    fn end_of_stmt(&mut self) {
        if self.done() || self.peek() == Some(&Tok::Nl) {
            return;
        }
        self.fail("more on this line than the statement wanted");
    }
}
