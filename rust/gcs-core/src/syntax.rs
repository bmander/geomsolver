//! Solvent syntax tree and source spans. Parsing, printing, and highlighting share these types.
//!
//! Only values inside `hint(…)` clauses are seeds a solve may write back.

mod highlight;
mod lexer;
mod names;
mod parser;
mod print;
mod source;
mod words;

pub use highlight::{highlight, Tint};
pub use names::{camel, entity_name, hidden, kind_initial, num, one_of, snake};
pub use parser::{parse, parse_from, parse_with_limits, ParseLimits};
pub use print::{operator_text, render_flat, write_stmt_to, PrintError};
pub use source::{line_col, Module, Name, Program, Span, StmtId, SynErr, Use, MAX_STMTS, MAX_TEXT};
pub use words::{equal_kind, is_name};

use crate::constraints::{CKind, Fixity};
use crate::model::EntKind;
use crate::style::{Classes, Style};
pub(crate) use names::{build_rank, decl_head, ref_text, under_root};
pub(crate) use print::{decl_args, decl_tail, hint_clause, hint_xy};

/// A component point traced over a numeric formal, with domain endpoints kept as expressions.
#[derive(Clone, Debug)]
pub struct CurveSpec {
    pub target: CurveTarget,
    /// The numeric formal that runs — `theta`.
    pub swept: Name,
    /// `in (a, b)`, as written.
    pub domain: (String, String),
    /// What the flattener resolved the target to — `None` until it has, or when it could not.
    pub of: Option<CurveOf>,
}

/// The instance a curve's point belongs to, resolved: the absolute prefix of the instance
/// (`leg.`, or a phantom `#c12.` for one written in place), and the point's name under it
/// (`toe`, `sub.pt`).
#[derive(Clone, Debug)]
pub struct CurveOf {
    pub instance: String,
    pub point: String,
}

/// Where a curve's point comes from — see `CurveSpec`.
#[derive(Clone, Debug)]
pub enum CurveTarget {
    /// `leg.toe`: a point of an instance the drawing holds.
    Drawn(Ref),
    /// `Leg(axle, pivot).toe`: an instance written in place, never drawn, and the point's path
    /// inside it.  The instance's name is a key the source cannot write.
    Anon(Instance, Ref),
}

#[derive(Clone, Debug, Default)]
pub struct Component {
    pub name: Option<Name>,
    pub formals: Vec<Formal>,
    pub body: Vec<Stmt>,
    pub span: Span,
    /// Which of the program's `modules` it was read from; `None` for one the document wrote.
    pub module: Option<usize>,
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
    /// A side of a line — `left` or `right`, the words a statement pins a magnitude with (§9.2).
    /// Not a number: a component that must place a point either side of an axis takes one of
    /// these, where before it took a `Scalar` it multiplied by (issue #48, item 4), which put the
    /// unreadable idiom inside every helper instead of at the statement.
    Side,
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
            "Side" => Ty::Side,
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
            Ty::Int | Ty::Scalar | Ty::Side | Ty::Ent(_) => crate::units::Dim::SCALAR,
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

/// How a desugared statement belongs to its written chain. Editors use this to
/// remove a relation without leaving an invalid joint or prefix.
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
    /// One word in a multiword joint. Deleting the last word applies `fall` to the
    /// whole joint; otherwise only this member is removed.
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
    /// A named traversal of the geometry declared by a chain expression.
    Chain(NamedChain),
    Relation(Relation),
    /// A recorded root choice under a key no triple of points spells — `branch(ppp:3|4|5, 1)`
    /// — kept verbatim so a document never silently loses one.  A choice that *is* a triple is
    /// written `ccw(a, b, c)`, a relation like any other (`CKind::Ccw`).
    Branch(Branch),
    /// `t: Tooth(root, tip, slot: 360 / N)` — a component, elaborated in place.
    Instance(Instance),
    /// `param R = m * N / 2` — a number worked out while elaborating, never an unknown.
    Param(ParamDecl),
    /// `repeat`, `cycle` — see `Block`.
    Block(Block),
    /// Parse a style block, retaining property spans for diagnostics.
    Style(StyleRule),
    /// A body operation (§6.9): `through` subtracts, `on` unites, `with` intersects.
    /// Relations are folded into the stock body after declarations are built.
    SolidRel(SolidRel),
    /// `claim over crank.theta in (0deg, 360deg) { … }` — the claims in the body, judged as the
    /// drawing runs along one of its own free variables (§9.8).  Structure-class: it says how the
    /// claims inside it are judged and asserts nothing itself.
    ClaimOver(ClaimOver),
    /// A derived view or section, expanded from its source body on the target plane.
    Derived(DerivedDecl),
    /// `unit mm` — what the document's numbers are in (spec §3.3).  A bare number in a `Length`
    /// slot is that unit, so every document keeps working with one added line; a document that
    /// says nothing is in **drawing units**, and everything still dimension-checks, you simply
    /// cannot write `mm` because there is nothing to convert to.
    Unit(Name),
}

/// `profile = line -> line -> line -> close`. The links remain declarations of the
/// enclosing scope; this value groups their traversal without copying their geometry.
#[derive(Clone, Debug)]
pub struct NamedChain {
    pub name: DeclName,
    pub links: Vec<Ref>,
    pub closed: bool,
}

/// Resolved style properties, their written order, and the declaration span.
#[derive(Clone, Debug)]
pub struct StyleRule {
    pub name: Name,
    pub style: Style,
    /// The property names as written, in order, so the printer says what the source said.
    pub props: Vec<String>,
    pub span: Span,
}

/// `claim over NAME in (a, b) { … }`.
#[derive(Clone, Debug)]
pub struct ClaimOver {
    pub formal: Ref,
    pub from: Arg,
    pub to: Arg,
    pub body: Vec<Stmt>,
    pub span: Span,
}

/// A picture asked of a solid (§6.11).
#[derive(Clone, Debug)]
pub struct DerivedDecl {
    pub name: DeclName,
    /// What it is a picture *of*.
    pub solid: Ref,
    /// The view it is drawn in.
    pub plane: Ref,
    /// A section's cutting plane; `None` for a plain view.
    pub at: Option<Ref>,
    /// `dimensions(body) in views.right` — **the sheet as a report** (§6.12): the callouts a
    /// machine can decide, laid out by the engine that lays out every other callout.
    pub dims: bool,
    pub class: Classes,
    pub span: Span,
}

/// Which side of the body rule a statement fills.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyWord {
    /// `boss on cyl` — material.
    On,
    /// A body operation (§6.9): `through` subtracts, `on` unites, `with` intersects.
    /// Relations are folded into the stock body after declarations are built.
    Through,
    /// `cylB.block.far against plate.body.near` — **a stack** (§6.10): two faces in contact, so
    /// where the left one's part stands is a *consequence* rather than a number somebody kept in
    /// step by hand.  What `zA = fwA + D / 2` was.
    Against,
}

impl BodyWord {
    pub fn as_str(self) -> &'static str {
        match self {
            BodyWord::On => "on",
            BodyWord::Through => "through",
            BodyWord::Against => "against",
        }
    }
}

/// A body operation (§6.9): `through` subtracts, `on` unites, `with` intersects.
/// Relations are folded into the stock body after declarations are built.
#[derive(Clone, Debug)]
pub struct SolidRel {
    pub word: BodyWord,
    pub what: Ref,
    pub body: Ref,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct Instance {
    pub name: Name,
    pub component: Name,
    pub args: Vec<InstArg>,
    pub span: Span,
    /// The view the instance is drawn in — `t: Tooth(…) in top` (§6.7): every point-bearing
    /// declaration its expansion makes joins the plane, the block's rule over the statements
    /// one statement stands for.  Carried into the expansion by the flattener
    /// (`Scope::in_plane`), never resolved here.
    pub membership: Membership,
    /// `t2: Throw(…) class phantom` — every declaration the expansion makes carries these
    /// classes under its own (§13.2), the way `in` puts the whole instance in a view.
    pub class: Classes,
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
pub struct ParamDecl {
    pub name: Name,
    pub text: String,
    pub span: Span,
}

/// Repetition.  Two constructs and two meanings (spec §12): the third, `ring`, is refused by
/// name until it can hold its copies congruent (issue #47, item 3).
#[derive(Clone, Debug)]
pub struct Block {
    pub kind: BlockKind,
    /// How many, as an expression over the enclosing parameters.
    pub count: String,
    /// `as i` — the index, available to every expression inside.
    pub binder: Option<Name>,
    pub body: Vec<Stmt>,
    /// The body's trailing open joint, where its last chain ends mid-joint: the chain threads
    /// onto the next copy (issue #38).
    pub joint: Option<OpenJoint>,
    pub span: Span,
}

/// A block body that ends mid-joint — `cycle N { distance(d) line -> angle(a) }` — threads its
/// chain onto the **next copy's** first link: every copy states the joint in a `cycle` or a
/// (the wrap seals the loop), and all but the last do in a `repeat`, whose final corner
/// is simply not stated.  Everything here is computed at parse time, where both links are
/// declarations of the body's own; what the flattener adds is only *which* copies.
#[derive(Clone, Debug)]
pub struct OpenJoint {
    /// The relations the joint states, one statement per word, desugared exactly as an
    /// in-chain joint's are — the right operand spelled `next.<first link>`, which the
    /// flattener's own `next` arm resolves per pair of copies.
    pub stmts: Vec<Stmt>,
    /// The words as written, for the printer — the statements above hold refs no source says.
    pub words: Vec<(String, Vec<OpArg>, Span)>,
    /// The chain's last link, whose exit the joint threads.
    pub last: OpenSide,
    /// The chain's first link, whose entry the next copy is entered by.
    pub first: OpenSide,
    /// Which side's slot names the shared point in the source.  At most one may (§6.6) — the
    /// parser refuses both, and this is that refusal carried as a fact rather than a pair of
    /// booleans a reader must remember cannot both be set.
    pub named: OpenNamed,
    /// The joint's own text — the marker through the last word — for errors about it.
    pub span: Span,
}

/// Which of an open joint's sides declares the shared point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpenNamed {
    /// Neither: the chain mints it, on the earlier-built side.
    Neither,
    /// The first link's entry names it (`line s(p) ->` — the point is the next copy's `p`).
    First,
    /// The last link's exit names it.
    Last,
}

/// One side of an open joint: enough to find the link's declaration in a copy's expansion and
/// state the weld there.
#[derive(Clone, Debug)]
pub struct OpenSide {
    /// The id of the link's declaration statement — the same id in every copy; the copy's
    /// instance path is what tells the clones apart.
    pub stmt: StmtId,
    pub kind: EntKind,
    /// Which child slot the thread fills — exit for the last link, entry for the first.
    pub slot: usize,
    /// The point where the chain crosses this side, body-relative: the slot's declared
    /// reference, or the dotted boundary path the mint would use (`<key>.p1`).  What the
    /// *other* side's slot is filled with, under `next.`/`prev.`.
    pub boundary: Ref,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockKind {
    /// N copies, and no relation between them.  `next`/`prev` are not in scope.
    Repeat,
    /// N copies that close: `next` is instance (i+1) mod N, `prev` is (i-1) mod N.
    Cycle,
}

impl BlockKind {
    /// Whether the copies close: `next` wraps, and a trailing open joint's last pair is the
    /// loop's closure.  `repeat` alone does not — the one rule, read wherever a pair or a
    /// sibling reference asks it, so the joint's statements and its welds cannot disagree on
    /// which pairs exist.
    pub fn wraps(self) -> bool {
        self != BlockKind::Repeat
    }
}

impl Block {
    /// The statements the block holds: its body's, and its trailing joint's — stated once, so
    /// a walker over the program cannot forget the joint's.
    pub fn stmts(&self) -> impl Iterator<Item = &Stmt> {
        self.body.iter().chain(self.joint.iter().flat_map(|j| j.stmts.iter()))
    }
}

/// Name visibility and writeback eligibility. All three forms resolve.
///
/// | Form | Displayed | Writable |
/// |---|---|---|
/// | Written (`l0`) | yes | yes |
/// | Copy (`#3.0.p`) | yes | no |
/// | No (`#a41`) | no | no |
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Named {
    /// The source wrote the name, and every prefix in front of it is a component instance's own
    /// name — so the whole dotted path is one a statement may be written with.
    #[default]
    Written,
    /// The source wrote the name, but a `cycle` or a `repeat` stands between: the flattener
    /// spells each copy `#3.0.p`, which says *which* copy — what a window shows and a selection
    /// is kept on, and stable, being the statement's id — and which carries a `#` no tokenizer
    /// will give back.
    Copy,
    /// The source wrote no name.  The parser minted a key — `#a` and the declaration's own
    /// offset — which resolves and is nothing else: not shown, not selected on, not written.
    No,
}

impl Named {
    /// Whether the source calls the thing this: an identity to publish, show and select by.
    pub fn shown(self) -> bool {
        self != Named::No
    }

    /// Whether a statement may be *written* with it.  Strictly narrower than `shown`, and a
    /// block's copy is the whole of the difference.
    pub fn writable(self) -> bool {
        self == Named::Written
    }
}

/// A resolution key paired with its display and writeback eligibility.
/// Preserve the variant when prefixing names during expansion.
#[derive(Clone, Debug)]
pub enum DeclName {
    /// The source wrote it, and every prefix the flattener put in front is a component
    /// instance's own name — so the whole dotted path is one a statement may be written with.
    Written(Name),
    /// The source wrote it, but a `cycle` or a `repeat` stands between: the flattener spells
    /// each copy `#3.0.p`, which says *which* copy — shown, selected by — and which carries a
    /// `#` no tokenizer will give back.
    Copy(Name),
    /// The source wrote none.  A minted resolution key — `#a` and the declaration's own offset,
    /// prefixed like any name when a block or a component encloses it — which resolves and is
    /// nothing else: not shown, not written, its span empty at the point a real name would go.
    Key(Name),
}

impl DeclName {
    /// What this declaration **resolves** by — every declaration has one, and a chain's corner
    /// welds by it.  Whether it is also what the source *calls* the thing is `shown`'s question,
    /// so a key must never reach the source, a report, or a reader's eye.
    pub fn key(&self) -> &Name {
        match self {
            DeclName::Written(n) | DeclName::Copy(n) | DeclName::Key(n) => n,
        }
    }

    /// What the source calls the thing, where it calls it anything — shown, published, selected
    /// by.  `None` for an anonymous declaration, whose key is nobody's to see.
    pub fn shown(&self) -> Option<&Name> {
        match self {
            DeclName::Written(n) | DeclName::Copy(n) => Some(n),
            DeclName::Key(_) => None,
        }
    }

    /// The narrower one: a name a statement may be **written** with.  A block's copy is shown
    /// and refused here, which is the whole difference between the two questions.
    pub fn written(&self) -> Option<&Name> {
        match self {
            DeclName::Written(n) => Some(n),
            _ => None,
        }
    }

    /// Where the name stands — or, an empty span, where one *would* go (`hint_span`'s device),
    /// which is where `edit::reconcile` splices a minted name.
    pub fn span(&self) -> Span {
        self.key().span
    }

    /// The three-question answer alone, which is `SourceMap::bind`'s vocabulary.
    pub fn named(&self) -> Named {
        match self {
            DeclName::Written(_) => Named::Written,
            DeclName::Copy(_) => Named::Copy,
            DeclName::Key(_) => Named::No,
        }
    }

    /// The same name under the prefix the flattener is putting on the front: an instance's own
    /// name keeps a written name writable, and a block's id (`copies`) makes any shown name one
    /// copy's.  A key stays a key — prefixed all the same, since two copies of one block hold
    /// two entities the resolver must tell apart.
    pub fn prefixed(&self, text: String, copies: bool) -> DeclName {
        let n = Name { text, span: self.span() };
        match self {
            DeclName::Key(_) => DeclName::Key(n),
            DeclName::Written(_) if !copies => DeclName::Written(n),
            _ => DeclName::Copy(n),
        }
    }
}

/// `point p0 hint(x: 0, y: 0)`, `circle c0(center: p2) hint(r: 25)`,
/// `spline s0(p3, p4, p5, p6) knots [...]`.
#[derive(Clone, Debug)]
pub struct Decl {
    pub kind: EntKind,
    /// The name **and what it is** — see `DeclName`.  Written where it is known first-hand and
    /// nowhere else: the parser either took an identifier or declined to (minting a key), and
    /// the flattener knows whether the prefix it is putting on the front is an instance's own
    /// name or a block's id.
    pub name: DeclName,
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
    /// Seed spans for writeback without reprinting. Empty for declarations built
    /// rather than parsed; expression seeds retain their text.
    pub seed_spans: Vec<Span>,
    /// The written hint clause, or its insertion point when absent.
    /// `None` marks synthetic declarations whose hints cannot be edited in source.
    pub hint_span: Option<Span>,
    /// Document data no solve moves, so not a seed and never written back.
    pub knots: Option<Vec<f64>>,
    /// A curve: what it is a curve *of* (§6.5).  `None` for every other kind.
    pub curve: Option<CurveSpec>,
    /// A **computed** point, `point p = (xexpr, yexpr)` (§6.5): its coordinates are expressions
    /// over the component's formals and params, and no constraint places it.  The brackets
    /// after a name say what the thing is made of, and this one is made of a formula — text,
    /// like a dimension's.  It is drawn only as a curve: a component with one is traced, never
    /// instantiated on the sheet.  `None` for every placed declaration.
    pub computed: Option<[(String, Span); 2]>,
    /// The classes it carries, in written order: `line l(a, b) class centerline heavy`.
    /// Presentation, and nothing the core computes reads it (spec §14).
    pub class: Classes,
    /// Where `class …` sits in the source, so a toggle rewrites the words and not the statement
    /// around them.  An *empty* span at the point one would be written when there is none.
    pub class_span: Span,
    /// A seed named *geometrically* rather than by coordinates: `hint(at: t)`,
    /// `hint(at: c.center)`, `hint(at: c, bearing: u + phase)`.  What it may name is the
    /// elaborator's question.
    pub seed_at: Option<AtRef>,
    /// The geometry the seed texts read (§6.4), each dotted name as written beside the absolute
    /// name of the entity it resolved to — filled by the flattener, read by the build, since an
    /// absolute name (`side.#282.0.small`) is not one the expression language can spell.
    pub seed_names: Vec<(String, String)>,
    /// A plane's attitude in space, as written (§6.7).  `Page` for every other kind.
    pub attitude: Attitude,
    /// How a solid is swept (§6.9): a prism along the plane's normal, a revolution about a line
    /// in it, or a body over other solids.  `None` for every other kind.
    pub sweep: Option<Sweep>,
    /// The plane this declaration's points are on — `point a in top`, and for a line, a circle,
    /// an arc, a spline or an ellipse, every point it mints or names (§6.7).  Its span is at
    /// the end of the trailers, so an appended clause lands after `hint`/`class` and never
    /// races `class_span`'s offset.
    pub membership: Membership,
    /// The `( … )` after the name — what the thing is made of, and a plane's attitude — or an
    /// empty span at the name's end when none was written.  `commit_seeds` replaces it rather
    /// than inserting a second list beside one that stated an attitude and no children.
    pub list_span: Span,
    /// An explicit closing edge (`-> close`). Its span includes the marker and word.
    pub close: Option<Span>,
}

/// Plane membership and its provenance (§6.7). Written clauses may be edited;
/// inherited membership must retain the block or declaration that supplied it.
#[derive(Clone, Debug, Default)]
pub struct Membership {
    plane: Option<Ref>,
    /// Where the clause is, or an empty span where one would go — `class_span`'s idiom.
    span: Span,
    from: Source,
}

/// Where a membership came from — see `Membership`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Source {
    /// The statement's own `in PLANE` clause.
    #[default]
    Written,
    /// The `in PLANE { … }` block around it.
    Block,
    /// An enclosing instance's `in PLANE` — the component's statements join the view whole.
    Instance,
}

impl Membership {
    /// The statement's own clause, as parsed.
    pub fn written_at(plane: Ref, span: Span) -> Membership {
        Membership { plane: Some(plane), span, from: Source::Written }
    }

    /// A membership a *lift* gives a statement it is about to print — written, so it prints,
    /// with no span of its own because there is no source behind it yet.
    pub fn lifted(plane: Ref) -> Membership {
        Membership::written_at(plane, Span::default())
    }

    /// Which plane, however the statement came by it: what resolution and the model ask.
    pub fn plane(&self) -> Option<&Ref> {
        self.plane.as_ref()
    }

    /// The same, to rescope — `flatten::rewrite` makes every reference absolute.
    pub fn plane_mut(&mut self) -> Option<&mut Ref> {
        self.plane.as_mut()
    }

    /// The clause the statement may **spell** — `None` where the plane came from a block or an
    /// enclosing instance, which wrote it once already.
    pub fn written(&self) -> Option<&Ref> {
        (self.from == Source::Written).then_some(self.plane.as_ref()).flatten()
    }

    /// Where the clause is, or would go.
    pub fn span(&self) -> Span {
        self.span
    }

    pub fn set_span(&mut self, span: Span) {
        self.span = span;
    }

    /// Put the statement in a plane it is not already in, saying where the clause came from;
    /// `false` when it is already in one, which is a plane given twice.
    pub fn join(&mut self, plane: &Ref, from: Source) -> bool {
        if self.plane.is_some() {
            return false;
        }
        self.plane = Some(plane.clone());
        self.from = from;
        true
    }

    /// Why a second plane is refused, in the words of whoever gave it the first.
    pub fn cause(&self) -> &'static str {
        match self.from {
            Source::Written => "already in a plane",
            Source::Block => "already in a plane: the block around it says which",
            Source::Instance => "already in a plane: the `in` on the instance says which",
        }
    }

    /// Whether an *edit* may write the clause here — a plane a block or an instance gave the
    /// statement is not this statement's to rewrite.
    pub fn editable(&self) -> bool {
        self.from == Source::Written
    }

    /// Who gave the statement its plane — the flattener asks, since a plane an *instance* gave
    /// was written in the caller's scope and resolves there, not in the component's.
    pub fn source(&self) -> Source {
        self.from
    }
}

/// An `in PLANE { … }` block's own text (§6.7): the header (`in PLANE {`) and the closing
/// brace.  The statements inside are the enclosing body's own — hoisted at parse, each stamped
/// with the plane — so nothing else remembers the block existed, and this is what `edit::remove`
/// splices when the plane goes: the header and the brace come out, and the statements stay.
#[derive(Clone, Debug)]
pub struct InBlock {
    pub plane: Ref,
    pub header: Span,
    pub close: Span,
}

/// What a plane's constant basis is made of, as written (§6.7): nothing (the page), another
/// plane and a fold, or the basis itself.  Never a seed — a solve moves none of it — which is
/// why it stands in the brackets with the children and not in `hint(…)` (§4.3).
#[derive(Clone, Debug, Default)]
pub enum Attitude {
    /// The page: the front view, `u = x`, `v = z`.
    #[default]
    Page,
    /// `from: front, fold: 30deg` — the plane perpendicular to `front` containing the direction
    /// at that bearing in it.  `fold` is an `Arg::Dim` (an Angle, as written); `None` is 0.
    From { plane: Ref, fold: Arg },
    /// A plane parallel to `plane`, displaced along its normal. An absent offset
    /// leaves placement to stack relations (§6.10).
    Offset { plane: Ref, offset: Option<Arg> },
    /// `u: (0.6, 0.8, 0), v: (0, 0, 1)` — six dimensionless `Arg::Dim`s.
    Basis { u: [Arg; 3], v: [Arg; 3] },
}

impl Attitude {
    /// The reference it names, if any — for the walks that fix every reference a statement
    /// makes (`flatten::rewrite`, `edit::mentions`).
    pub fn plane_ref(&self) -> Option<&Ref> {
        match self {
            Attitude::From { plane, .. } | Attitude::Offset { plane, .. } => Some(plane),
            Attitude::Page | Attitude::Basis { .. } => None,
        }
    }

    pub fn plane_ref_mut(&mut self) -> Option<&mut Ref> {
        match self {
            Attitude::From { plane, .. } | Attitude::Offset { plane, .. } => Some(plane),
            Attitude::Page | Attitude::Basis { .. } => None,
        }
    }

    /// Every number it was written over, for the flattener to settle a component's parameters
    /// into.
    pub fn args_mut(&mut self) -> Vec<&mut Arg> {
        match self {
            Attitude::Page => Vec::new(),
            Attitude::From { fold, .. } => vec![fold],
            Attitude::Offset { offset, .. } => offset.iter_mut().collect(),
            Attitude::Basis { u, v } => u.iter_mut().chain(v.iter_mut()).collect(),
        }
    }
}

/// A prism, revolution, or body operation (§6.9). Numeric arguments remain
/// expressions until elaboration.
#[derive(Clone, Debug)]
pub enum Sweep {
    /// `from: a, to: b` — signed ordinates along the plane's normal.
    Prism { from: Arg, to: Arg },
    /// A positive magnitude, kept until elaboration can validate its evaluated expression.
    Depth { depth: Arg },
    /// `about: ax` — a full turn about a line in the face's own plane, or `sweep:` of one,
    /// `sense: cw` the other way round.
    Revolve { axis: Ref, sweep: Option<Arg>, sense: Sense },
    /// `solid body(block)` — a stock, or a term: what it is made of is in the list, and the
    /// `on`/`through` statements say the rest.
    Body,
}

impl Sweep {
    /// Every number it was written over, for the flattener to settle a component's parameters
    /// into — the same walk `Attitude::args_mut` joins, and for the same reason: an extent is
    /// written in the little language a dimension is, and a `param` is in scope for it.
    pub fn args_mut(&mut self) -> Vec<&mut Arg> {
        match self {
            Sweep::Prism { from, to } => vec![from, to],
            Sweep::Depth { depth } => vec![depth],
            Sweep::Revolve { sweep, .. } => sweep.iter_mut().collect(),
            Sweep::Body => Vec::new(),
        }
    }

    /// The line a revolution turns about, for the walks that fix every reference a statement
    /// makes.
    pub fn axis_ref_mut(&mut self) -> Option<&mut Ref> {
        match self {
            Sweep::Revolve { axis, .. } => Some(axis),
            Sweep::Prism { .. } | Sweep::Depth { .. } | Sweep::Body => None,
        }
    }

    pub fn axis_ref(&self) -> Option<&Ref> {
        match self {
            Sweep::Revolve { axis, .. } => Some(axis),
            Sweep::Prism { .. } | Sweep::Depth { .. } | Sweep::Body => None,
        }
    }
}

/// Which way a partial revolution turns.  **A word, never a sign** (§9.2): a negative sweep is
/// refused where it is written, because `sweep(-90deg)` says nothing a reader can picture.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Sense {
    #[default]
    Ccw,
    Cw,
}

/// A named child, an anonymous point seed, or a face written inside a solid. Only chain
/// desugaring may leave a partial child list; threading fills it before elaboration.
/// `D` is a syntax declaration here and an IR declaration after lowering.
#[derive(Clone, Debug)]
pub enum Kid<D = Decl> {
    /// `line l(a, b)` — the point is named, and named somewhere else.
    Ref(Ref),
    /// `line l(hint(x: 0, y: 0), …)` — an anonymous point, and where its solve begins.  The
    /// same clause as everywhere else in the language, one level down.
    Hint(KidSeed),
    /// `solid block(face(a, b, c, -> close), depth: t)` — a private section.
    Face { decl: Box<D>, span: Span },
}

impl<D> Kid<D> {
    pub fn as_ref(&self) -> Option<&Ref> {
        match self {
            Kid::Ref(r) => Some(r),
            Kid::Hint(_) | Kid::Face { .. } => None,
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

/// `hint(at: c, bearing: u + phase)` — a place given as geometry: at a point, or at the edge
/// of a circle at a bearing from the page's x-axis.  The `at:` and `bearing:` keys of the one
/// seed clause (§6.4), read out of it beside the scalars.
#[derive(Clone, Debug)]
pub struct AtRef {
    pub what: Ref,
    pub bearing: Option<(String, Span)>,
}

/// One written operator argument: a selector, entity, owned slot, or dimension.
#[derive(Clone, Debug)]
pub enum OpArg {
    /// `side: -1`, `at: start`, `along: x`, `external: true`
    Named(Name, Arg),
    /// the third entity, unlabelled: `a symmetry(l) b` — or every operand of a call, `ccw(a, b, c)`
    Ent(Ref),
    /// A named constraint slot. Pins (`t == 0.4`) constrain the solution; values
    /// inside `hint(…)` only seed it. Selectors such as `end: start` use `Named`.
    Slot { key: Name, arg: Arg },
    /// the number, as written — `80`, `x = 7`, `h = w / 2`, `1' 3"`
    Dim(String, Span),
}

/// An operator spelling retained through resolution, with operands and arguments
/// in written order for printing and source edits.
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

    /// Where a selector's key was written — what a diagnostic about its *value* points at, since
    /// a value carries no span of its own (`Arg::Word` is a bare `String`).
    pub fn key_span(&self, name: &str) -> Option<Span> {
        self.args.iter().find_map(|a| match a {
            OpArg::Named(n, _) if n.text == name => Some(n.span),
            _ => None,
        })
    }

    /// Assemble arguments in registry order, rejecting unknown slot and selector names.
    /// Missing arguments remain `None` for elaboration to validate.
    pub fn assemble(&self, kind: CKind) -> Result<Vec<Option<Arg>>, (Span, String)> {
        let spec = kind.spec();
        // a seed or a pin names the slot it fills, and a name the kind does not have is a typo
        // rather than something to fill the first slot with: filled by position, the wrong
        // word here would silently pin the right slot at the wrong
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
        // and a *selector* naming nothing was dropped in silence, which is the same mistake one
        // layer down (issue #48, item 4): `a distance(80, sied: x) b` settled as a plain distance
        // and the argument went nowhere.  `along` is the one key with no slot — it chose the kind
        // and is gone — so it is named here rather than looked for in the spec.
        for a in &self.args {
            let OpArg::Named(key, _) = a else { continue };
            if key.text != "along" && !spec.iter().any(|(n, _)| *n == key.text) {
                let word = &self.word.text;
                let m = format!("`{word}` takes no `{}`", key.text);
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
            out[i] = if sk.takes_ref() {
                next.next().map(Arg::Ref)
            } else if sk.is_param() {
                self.args.iter().find_map(|a| match a {
                    OpArg::Slot { key, arg } if key.text == *name => Some(arg.clone()),
                    _ => None,
                })
            } else if sk.is_dimension() {
                // the number, wherever the dimension slot stands: a kind has at most one, so it
                // is *the* number in the parentheses, and a selector may follow it in spec order
                // (`distance(12, side: left)` — issue #48, item 4)
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
    pub form: RelationForm,
    /// Where the callout was dragged to, if anywhere.  A seed: inert, and written back.
    pub place: Option<(f64, f64)>,
    /// Where `at (t, r)` sits in the source, so a callout dragged somewhere else rewrites those
    /// characters instead of the statement around them.  Empty for a relation that was built
    /// rather than parsed, and for one that carries no placement — in both cases there is no
    /// text yet, and the writeback appends after the statement.
    pub place_span: Span,
    /// Written `claim …` (§9.7): stated as expected to add no rank, judged by the diagnosis and
    /// never solved for.
    pub claim: bool,
    /// The classes the statement carries (`a distance(80) b class ref`), which is how a
    /// dimension's callout is given a look of its own — or none, under `display: none` — the
    /// way a declaration's is (§13.2).  A relation that states no dimension draws nothing, and a
    /// class on it is inert.
    pub class: Classes,
    /// Where the clause is, or an empty span where one would go.
    pub class_span: Span,
}

/// Parsed operators and generated registry calls are mutually exclusive.
#[derive(Clone, Debug)]
pub enum RelationForm {
    Written(Written),
    Canonical { kind: CKind, args: Vec<Option<Arg>> },
}

impl RelationForm {
    pub fn written(&self) -> Option<&Written> {
        match self {
            Self::Written(w) => Some(w),
            Self::Canonical { .. } => None,
        }
    }

    pub fn written_mut(&mut self) -> Option<&mut Written> {
        match self {
            Self::Written(w) => Some(w),
            Self::Canonical { .. } => None,
        }
    }

    pub fn canonical_args(&self) -> &[Option<Arg>] {
        match self {
            Self::Canonical { args, .. } => args,
            Self::Written(_) => &[],
        }
    }

    pub fn canonical_args_mut(&mut self) -> &mut [Option<Arg>] {
        match self {
            Self::Canonical { args, .. } => args,
            Self::Written(_) => &mut [],
        }
    }
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
    Dim {
        text: String,
        span: Span,
    },
    /// A named constraint slot. Pins (`t == 0.4`) constrain the solution; values
    /// inside `hint(…)` only seed it. Selectors such as `end: start` use `Named`.
    Seed {
        value: f64,
        pinned: bool,
    },
    /// The same, written over the parameters in scope — `u = u0` inside a component.  Worked out
    /// during expansion and a plain `Seed` from then on.
    SeedExpr {
        text: String,
        pinned: bool,
        span: Span,
    },
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
        Ref { root: Name::new(name), path: vec![Seg::Field(Name::new(f))], span: Span::default() }
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
/// `branch(KEY, ±1)` — a recorded root choice under a key `decompose::branch_key_points` could
/// not read as a triple of points.
#[derive(Clone, Debug)]
pub struct Branch {
    pub key: String,
    pub value: i32,
}

/// Preserve a seed as a literal or expression, with its span and pin status.
fn seed_arg(value: Option<f64>, text: String, span: Span, pinned: bool) -> Arg {
    match value {
        Some(value) => Arg::Seed { value, pinned },
        None => Arg::SeedExpr { text, pinned, span },
    }
}
