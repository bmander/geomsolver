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
pub use parser::{parse, parse_from, parse_legacy, parse_with_limits, ParseLimits};
pub use print::{bounded, operator_text, render_flat, write_stmt_to, PrintError};
pub(crate) use print::{sel_text, written_parts};
pub use source::{line_col, Module, Name, Program, Span, StmtId, SynErr, Use, MAX_STMTS, MAX_TEXT};
pub use words::{equal_kind, is_name, reserved_word};

use crate::constraints::{CKind, Fixity};
use crate::model::EntKind;
use crate::style::{Classes, Style};
pub(crate) use names::{build_rank, decl_head, ref_text, under_root};
pub(crate) use print::{decl_args, hint_clause, hint_numbers, said, said_text, Said};

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
    /// `component Sphere(center: point, r: Length) := { p | p distance(r) center }` — a family
    /// of sets (§6.21): an instance *is* the set, its formals its named parts.  The body is
    /// empty; what being on the set means is the literal's.
    pub set: Option<SetLit>,
}

/// **A set, written as the points that satisfy a predicate** (§6.21): `{ p | p distance(12mm)
/// c }`.  Making one adds nothing to the drawing; `q coincident S` is the body with `q` for
/// `p`, and `l tangent S` the body at a contact on `l` with its derivative along `l`
/// (`flatten::sets`).
#[derive(Clone, Debug)]
pub struct SetLit {
    /// The point the body is about — `p` — bound to the operand at each use.
    pub bound: Name,
    pub body: Vec<Stmt>,
    pub span: Span,
}

/// `ball := { p | … }` — a set written in place, reading the names of the body it stands in.
#[derive(Clone, Debug)]
pub struct SetDecl {
    pub name: Name,
    pub lit: SetLit,
}

/// **A relation stated as its derivative** (§6.21): the flattener's twin of a set's body row for
/// a tangency — the row's rate as `point` moves (along a line's direction, or in a chart), the
/// geometry the use made moving with it and every other number of the row held.  `key` names the
/// use's derivative, one per tangency (one per chart direction of a tangency at a point); `made`
/// is the prefix the use's own geometry is made under, where it may make some.  Never written;
/// the references are the expansion's own.
#[derive(Clone, Debug)]
pub struct Along {
    pub point: Ref,
    pub toward: AlongBy,
    pub key: String,
    pub made: Option<String>,
}

/// What moves an `Along`'s point.
#[derive(Clone, Debug)]
pub enum AlongBy {
    /// `l tangent S`: the line's direction.
    Line(Ref),
    /// `S1 tangent(at: m) S2`: the `k`th of two directions solved for.
    Chart(u8),
}

/// **A relation word defined in the language** (§9.9): `a horizontal b := a level(up) b`, `flat
/// l := l perpendicular t`, `a above(d) b := b distance(d, along: up) a`.  Written at the top of
/// a file, as a component is; a statement writing the word is the body with the operands and the
/// parameters put in, expanded by the flattener where the statement stands
/// (`flatten::words`).
#[derive(Clone, Debug)]
pub struct WordDef {
    pub word: Name,
    /// `Infix` over two operands or `Prefix` over one, as the head is written.
    pub fixity: Fixity,
    /// The operands' names, in written order — what the body's references may name.
    pub operands: Vec<Name>,
    /// The parameters in the word's parentheses: numbers or selector words, given by label
    /// where the word is written (§4.1).
    pub params: Vec<Name>,
    /// What the word stands for, as written after `:=`: one relation, or a braced body of
    /// relations and the declarations they need (#103) — a fact about the two operands that takes
    /// several rows to state.
    pub body: Vec<Stmt>,
    pub span: Span,
    /// Which of the program's `modules` it was read from; `None` for one the document wrote.
    pub module: Option<usize>,
}

/// **A defined word, as the statement using it wrote it** (§9.9): carried on the relation its
/// body expands to, so a constraint is described in the word and not in its expansion.  The
/// operands (two of an infix word) are rescoped like any reference; the arguments are the
/// parentheses' text.
#[derive(Clone, Debug)]
pub struct Worded {
    pub word: String,
    pub ops: Vec<Ref>,
    /// `(d: 5mm)`, as written, or empty.
    pub args: String,
    /// The operands that are sets (§6.21), each by its place among the operands and as the
    /// statement wrote it — `ball` in `q coincident ball`, both in `gc tangent(at: m) pc`: no
    /// entity, so they stand outside `ops`, described by the name and never resolved.
    pub sets: Vec<(usize, String)>,
    /// The word as the statement wrote it, where a fault in what it expanded to is said.
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
    /// A side of a line — `left` or `right`, the words a statement pins a magnitude with (§9.2).
    /// Not a number: a component that must place a point either side of an axis takes one of
    /// these, where before it took a `Scalar` it multiplied by (issue #48, item 4), which put the
    /// unreadable idiom inside every helper instead of at the statement.
    Side,
    /// A named bundle of values and geometry aliases, or a component's layout instance.
    Group,
    Ent(EntKind),
}

/// The words the types other than an entity kind are written as — one table, read both ways.
const TYPE_WORDS: [(&str, Ty); 6] = [
    ("Int", Ty::Int),
    ("Scalar", Ty::Scalar),
    ("Length", Ty::Length),
    ("Angle", Ty::Angle),
    ("Side", Ty::Side),
    ("group", Ty::Group),
];

impl Ty {
    /// The word a type is written as.  One table: a formal's type, a curve family's, and the
    /// colouring of the word are the same question, and a second copy of the list would be a
    /// second answer the moment a type is added.
    pub fn parse(s: &str) -> Option<Ty> {
        match TYPE_WORDS.iter().find(|(w, _)| *w == s) {
            Some(&(_, ty)) => Some(ty),
            None => Some(Ty::Ent(EntKind::parse(&s.to_lowercase())?)),
        }
    }

    /// The word `parse` reads a type from, by the same table (`None` for an entity kind).
    pub fn word(self) -> Option<&'static str> {
        TYPE_WORDS.iter().find(|(_, ty)| *ty == self).map(|&(w, _)| w)
    }

    /// The number type of a dimension (`units.rs`): `Length`, `Angle`, else `Scalar`.
    pub fn of_dim(d: crate::units::Dim) -> Ty {
        match d {
            d if d == crate::units::Dim::LENGTH => Ty::Length,
            d if d == crate::units::Dim::ANGLE => Ty::Angle,
            _ => Ty::Scalar,
        }
    }

    /// What a formal declared this way *is* (`units.rs`) — the same table one question further
    /// on, and here for `parse`'s reason: a second copy of the list would be a second answer the
    /// moment a type is added.  `Length` and `Angle` name the two base dimensions; `Int` and
    /// `Scalar` are plain numbers, and an entity formal is not a number at all.
    pub fn dim(self) -> crate::units::Dim {
        match self {
            Ty::Length => crate::units::Dim::LENGTH,
            Ty::Angle => crate::units::Dim::ANGLE,
            Ty::Int | Ty::Scalar | Ty::Side | Ty::Group | Ty::Ent(_) => crate::units::Dim::SCALAR,
        }
    }

    /// Whether a formal of this type is a number — bound to one, or an unknown where left unbound.
    pub fn number(self) -> bool {
        matches!(self, Ty::Int | Ty::Scalar | Ty::Length | Ty::Angle)
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
    /// `t := Tooth(root, tip, slot: 360 / N)` — a component, elaborated in place.
    Instance(Instance),
    /// `R := m * N / 2` — a number worked out while elaborating, never an unknown.
    Param(ParamDecl),
    /// `dims := {bore: 16mm, axis: datum}` — named values, without new geometry.
    Group(GroupDecl),
    /// `repeat`, `cycle` — see `Block`.
    Block(Block),
    /// Parse a style block, retaining property spans for diagnostics.
    Style(StyleRule),
    /// A body operation (§6.9): `cut` subtracts, `union` unites, `bound` intersects.
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
    /// `ball := { p | p distance(12mm) c }` — a set (§6.21), see `SetLit`.
    Set(SetDecl),
    /// `rope minimizes integral(p.y over p)` — an energy over a curve (#121): the curve takes
    /// the shape that makes it stationary.
    Minimize(Minimize),
}

impl StmtKind {
    /// The name a statement defines in its body — a declaration, a chain, an instance, a value or
    /// a group — and so the one no other name there may take.
    pub fn bound_name(&self) -> Option<&Name> {
        match self {
            StmtKind::Decl(d) => Some(d.name.key()),
            StmtKind::Chain(c) => Some(c.name.key()),
            StmtKind::Instance(i) => Some(&i.name),
            StmtKind::Param(d) => Some(&d.name),
            StmtKind::Group(g) => Some(&g.name),
            StmtKind::Set(d) => Some(&d.name),
            _ => None,
        }
    }
}

/// `profile := line -> line -> line -> close`. The links remain declarations of the
/// enclosing scope; this value groups their traversal without copying their geometry.
#[derive(Clone, Debug)]
pub struct NamedChain {
    pub annotations: crate::semantics::Annotations,
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

/// `k minimizes TERM + TERM …` (or `maximizes`): a sum of integrals along the curve `k`, each
/// with a constant coefficient (#121).  A statement of what the curve is, not an instruction: its
/// shape is stationary for the energy, and that the stationary shape is the extremum the word
/// names is judged, as a claim is (`Diagnosis::extrema`).
#[derive(Clone, Debug)]
pub struct Minimize {
    pub curve: Ref,
    pub maximize: bool,
    pub terms: Vec<Integral>,
    pub span: Span,
}

/// `c * integral(EXPR over p)` or `… over (p, t)`: `EXPR` read at the point `p` running along
/// the statement's curve (and `t`, its unit tangent there), weighted by arc length.
#[derive(Clone, Debug)]
pub struct Integral {
    pub coef: f64,
    pub point: Name,
    pub tangent: Option<Name>,
    pub body: String,
    pub body_span: Span,
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
    /// `boss union cyl` — material.
    Union,
    /// `bore cut cyl` — the body rule's subtraction (§6.9). Relations are folded into
    /// the stock body after declarations are built.
    Cut,
    /// `tip bound cyl` — the body rule's intersection: the body keeps what lies within
    /// every solid that bounds it.
    Bound,
}

impl BodyWord {
    pub fn as_str(self) -> &'static str {
        match self {
            BodyWord::Union => "union",
            BodyWord::Cut => "cut",
            BodyWord::Bound => "bound",
        }
    }
}

/// A body operation (§6.9): `cut` subtracts, `union` unites, `bound` intersects.
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
    pub annotations: crate::semantics::Annotations,
    /// A written instance name, or a private `#i…` key for an unnamed call.
    pub name: Name,
    pub component: Name,
    pub args: Vec<InstArg>,
    pub span: Span,
    /// The view the instance is drawn in — `t := Tooth(…) in top` (§6.7): every point-bearing
    /// declaration its expansion makes joins the plane, the block's rule over the statements
    /// one statement stands for.  Carried into the expansion by the flattener
    /// (`Scope::in_plane`), never resolved here.
    pub membership: Membership,
    /// `t2 := Throw(…) class phantom` — every declaration the expansion makes carries these
    /// classes under its own (§13.2), the way `in` puts the whole instance in a view.
    pub class: Classes,
}

impl Instance {
    /// Where the call writes the number it gives `formal` (`r: 25`'s `25`), labelled and worked
    /// out here — what a dimension reading the formal is drawn and edited at.
    pub fn given(&self, formal: &str) -> Option<Span> {
        self.args.iter().find_map(|a| match (&a.label, &a.value) {
            (Some(l), InstVal::Expr(t)) if l.text == formal => {
                Some(Span::new(a.span.hi as usize - t.len(), a.span.hi as usize))
            }
            _ => None,
        })
    }
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
    /// A group written in place as a group's member, `{bore: 16mm, axis: datum}`: its members
    /// are the outer group's under the member's name (`dims.cyl.bore`), and the member is a
    /// group a call may be given (`Part(dims.cyl)`).  Only a group's member may be one.
    Group(Vec<InstArg>),
    /// `beta: hint(15deg)` — the formal left unbound, so an unknown of the drawing as any
    /// unbound numeric formal is, and its solve begun at this number, which a solve writes back
    /// at the span.  Only a call's argument.
    Hint(String, Span),
}

/// A number defined by name: `w := 60`, a value worked out while elaborating, or — under the
/// word `param` — one of the document's **inputs** (§6.3): `param bore: Length := 50mm`, which a
/// host may give another value, or `param beta: Angle hint(30deg)`, which nothing binds and is
/// therefore an unknown of the solve, as a component's unbound formal is.
#[derive(Clone, Debug)]
pub struct ParamDecl {
    pub name: Name,
    /// The value after `:=`; empty for an input nothing binds.
    pub text: String,
    /// Where the value is; for an unbound input, the empty span after the declaration's type.
    pub span: Span,
    /// `Some` under the word `param`.
    pub input: Option<Input>,
}

/// What `param` adds to a definition: its declared type and, for one nothing binds, its seed.
#[derive(Clone, Debug)]
pub struct Input {
    pub ty: Option<Ty>,
    /// `hint(30deg)`: the seed's text and span, only on an input nothing binds.
    pub seed: Option<(String, Span)>,
}

impl ParamDecl {
    /// Whether the definition gives the name a value — every plain definition does; an input
    /// may not, and is then an unknown.
    pub fn bound(&self) -> bool {
        !self.text.is_empty()
    }
}

#[derive(Clone, Debug)]
pub struct GroupDecl {
    pub name: Name,
    pub fields: Vec<InstArg>,
}

/// Repetition.  Three constructs and three meanings (spec §12): an open array, a closed one, and
/// a ring — a closed one whose copies are one another's turns (issue #96).
#[derive(Clone, Debug)]
pub struct Block {
    pub kind: BlockKind,
    /// How many, as an expression over the enclosing parameters.  Empty where the block runs
    /// over a named chain's edges (`over`), whose count is the chain's own.
    pub count: String,
    /// `repeat e in rack.profile { … }` — one copy per edge of a named chain, in traversal order,
    /// with `e` naming that copy's edge (§6.6, §12).
    pub over: Option<EdgesOf>,
    /// `as i` — the index, available to every expression inside.
    pub binder: Option<Name>,
    pub body: Vec<Stmt>,
    /// The body's trailing open joint, where its last chain ends mid-joint: the chain threads
    /// onto the next copy (issue #38).
    pub joint: Option<OpenJoint>,
    /// A ring's centre, `ring N about C`: a point, or an axis (§12.3).  `None` for the others.
    pub about: Option<Ref>,
    /// Where a ring's body reads its own index — E015, said by the flattener: every copy is the
    /// representative turned, so there is nothing an index could vary.
    pub index_reads: Vec<Span>,
    pub span: Span,
}

/// What a block runs over when it runs over a chain: the name each copy calls its edge, and the
/// chain as written.  The chain is resolved by the flattener in the scope the block stands in,
/// so it may be reached through an instance or a formal (`refs.pinion.profile`).
#[derive(Clone, Debug)]
pub struct EdgesOf {
    pub var: Name,
    pub chain: Ref,
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
    /// N copies that close and are one another's turns about a centre (§12.3): solved over the
    /// first, the representative, the others worked out from it (`model::Turn`).
    Ring,
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

/// One weight of a rational spline: its value, and its text where it was written as an expression
/// (`(1 + sqrt(2)) / 3`), worked out during expansion against the scope's values and `None` from
/// then on, as a seed's is.
#[derive(Clone, Debug, PartialEq)]
pub struct Weight {
    pub value: f64,
    pub text: Option<String>,
    pub span: Span,
}

/// `p0 := point hint((0, 0))`, `c0 := circle(center: p2) hint(r: 25)`,
/// `spline s0(p3, p4, p5, p6) knots [...]`.
#[derive(Clone, Debug)]
pub struct Decl {
    pub annotations: crate::semantics::Annotations,
    pub kind: EntKind,
    /// The name **and what it is** — see `DeclName`.  Written where it is known first-hand and
    /// nowhere else: the parser either took an identifier or declined to (minting a key), and
    /// the flattener knows whether the prefix it is putting on the front is an instance's own
    /// name or a block's id.
    pub name: DeclName,
    /// One per `Child`/`List` field of `EntKind::fields`, in that order; a `List` field holds as
    /// many as were written.  Empty throughout — `l := line` — is the anonymous form: the kind's
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
    /// A rational spline's weights, one per control point (`weights [1, w, w, 1]`): document data
    /// like the knots, each written as a number or an expression over the scope's values.
    pub weights: Option<Vec<Weight>>,
    /// A curve: what it is a curve *of* (§6.5).  `None` for every other kind.
    pub curve: Option<CurveSpec>,
    /// A **computed** point, `p := point(x: xexpr, y: yexpr)` (§6.5): its coordinates are
    /// expressions over the component's formals and params, and no constraint places it.  The
    /// brackets say what the thing is made of, and this one is made of a formula — text, like a
    /// dimension's.  It is drawn only as a curve: a component with one is traced, never
    /// instantiated on the sheet.  `None` for every placed declaration.
    pub computed: Option<[(String, Span); 2]>,
    /// The classes it carries, in written order: `l := line(a, b) class centerline heavy`.
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
    /// How a solid is swept (§6.9): a prism along the plane's normal, a revolution about a line
    /// in it, or a body over other solids.  `None` for every other kind.
    pub sweep: Option<Sweep>,
    pub motion: Option<MotionSpec>,
    /// A source surface's angular span or an envelope's roll interval.
    pub angular_span: Option<AngularSpan>,
    /// The plane this declaration's points are on — `a := point in top`, and for a line, a circle,
    /// an arc, a spline or an ellipse, every point it mints or names (§6.7).  Its span is at
    /// the end of the trailers, so an appended clause lands after `hint`/`class` and never
    /// races `class_span`'s offset.
    pub membership: Membership,
    /// The `( … )` after the name — what the thing is made of — or an empty span at the name's
    /// end when none was written.
    pub list_span: Span,
    /// An explicit closing edge (`-> close`). Its span includes the marker and word.
    pub close: Option<Span>,
    /// Where the `)` goes when a name is minted for this anonymous declaration: a chain link
    /// is named in parentheses, `(l := line(a, b)) -> …`, since `:=` binds looser than `->`.
    /// `None` where `name := ` alone stands at the name's span — a statement's whole value.
    pub mint_close: Option<usize>,
}

impl Decl {
    /// A point built rather than parsed: seeded at `seed`, or at a place, with no clause in any
    /// text to write a solve back into.
    pub fn point(name: DeclName, seed: [f64; 2], seed_at: Option<AtRef>) -> Decl {
        Decl {
            annotations: Default::default(),
            kind: EntKind::Point,
            name,
            children: Vec::new(),
            seed: seed.to_vec(),
            seed_text: vec![None; 2],
            seed_spans: Vec::new(),
            hint_span: None,
            knots: None,
            weights: None,
            curve: None,
            computed: None,
            class: Default::default(),
            class_span: Span::default(),
            seed_at,
            seed_names: Vec::new(),
            sweep: None,
            motion: None,
            angular_span: None,
            membership: Membership::default(),
            list_span: Span::default(),
            close: None,
            mint_close: None,
        }
    }

    /// `k := leg.toe over theta in (0, 360)` — the declaration a curve definition makes.
    pub fn curve(name: DeclName, curve: CurveSpec, class: Classes, class_span: Span) -> Decl {
        Decl {
            annotations: Default::default(),
            kind: EntKind::Curve,
            name,
            children: Vec::new(),
            seed: Vec::new(),
            seed_text: Vec::new(),
            seed_spans: Vec::new(),
            hint_span: None,
            knots: None,
            weights: None,
            curve: Some(curve),
            class,
            class_span,
            computed: None,
            seed_at: None,
            seed_names: Vec::new(),
            sweep: None,
            motion: None,
            angular_span: None,
            membership: Membership::default(),
            list_span: Span::default(),
            close: None,
            mint_close: None,
        }
    }
}

/// Plane membership and its provenance (§6.7). Written clauses may be edited;
/// inherited membership must retain the block or declaration that supplied it.
#[derive(Clone, Debug, Default)]
pub struct Membership {
    plane: Option<Ref>,
    /// The further planes a point is drawn in (`point in P, G`, §6.7): it is on each, so where
    /// they meet, and drawn in each by a twin of its own (`Sketch::twins`).
    also: Vec<Ref>,
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
        Membership { plane: Some(plane), also: Vec::new(), span, from: Source::Written }
    }

    /// The same, on further planes too (`in P, G`).
    pub fn written_on(plane: Ref, also: Vec<Ref>, span: Span) -> Membership {
        Membership { also, ..Membership::written_at(plane, span) }
    }

    /// The further planes, after the first (`in P, G`'s `G`).
    pub fn also(&self) -> &[Ref] {
        &self.also
    }

    /// The same, to rescope.
    pub fn also_mut(&mut self) -> &mut [Ref] {
        &mut self.also
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

/// A declared angular domain; it is data, not a solver unknown or hint.
#[derive(Clone, Debug)]
pub struct AngularSpan {
    pub from: Arg,
    pub to: Arg,
    /// Which cut a planar envelope takes where its tool cuts twice (`side: near | far` of the
    /// instant centre, §6.15.1); a word, checked where the envelope is built.
    pub side: Option<Name>,
}

/// A rigid rotation (a screw with `advance:`), a translation, or a relative
/// motion, over one shared angular parameter.
#[derive(Clone, Debug)]
pub enum MotionSpec {
    Rotation { axis: Ref, ratio: Option<Arg>, phase: Option<Arg>, advance: Option<Arg> },
    Translation { axis: Ref, advance: Arg },
    Relative { source: Ref, observer: Ref },
}

impl MotionSpec {
    pub fn refs_mut(&mut self) -> Vec<&mut Ref> {
        match self {
            Self::Rotation {axis,..} | Self::Translation {axis,..} => vec![axis],
            Self::Relative {source,observer} => vec![source,observer],
        }
    }

    pub fn args_mut(&mut self) -> Vec<&mut Arg> {
        match self {
            Self::Rotation {ratio,phase,advance,..} =>
                ratio.iter_mut().chain(phase.iter_mut()).chain(advance.iter_mut()).collect(),
            Self::Translation {advance,..} => vec![advance],
            Self::Relative {..} => vec![],
        }
    }
}

/// A prism, revolution, or body operation (§6.9). Numeric arguments remain
/// expressions until elaboration.
#[derive(Clone, Debug)]
pub enum Sweep {
    /// A solid placed at one angle of a named rigid motion.
    Placed { motion: Ref, at: Arg },
    /// The continuous union of a solid under a finite motion interval.
    Swept { motion: Ref, from: Arg, to: Arg },
    /// `from: a, to: b` — signed ordinates along the plane's normal.
    Prism { from: Arg, to: Arg },
    /// One section repeated, or two sections interpolated, along a directed guide.
    Along { guide: Ref },
    /// A positive magnitude, kept until elaboration can validate its evaluated expression.
    Depth { depth: Arg },
    /// Span the target's stock and additions along the section normal.
    Through { body: Ref },
    /// `about: ax` — a full turn about a line in the face's own plane, or `sweep:` of one,
    /// `sense: cw` the other way round.
    Revolve { axis: Ref, sweep: Option<Arg>, sense: Sense },
    /// `body := solid(block)` — a stock, or a term: what it is made of is in the list, and the
    /// `union`/`cut`/`bound` statements say the rest.
    Body,
    /// `blank := solid(R)`, R a set — the points inside it (§6.21): written as a `Body` over the
    /// set, and made this by the flattener, which applies the set to the hidden `probe` point
    /// (`probe inside R`) and whose bounds the elaborator reads as the solid's terms.
    Region { probe: Ref },
}

impl Sweep {
    /// Every number it was written over, for the flattener to settle a component's parameters
    /// into: an extent is written in the little language a dimension is, and a `param` is in
    /// scope for it.
    pub fn args_mut(&mut self) -> Vec<&mut Arg> {
        match self {
            Sweep::Placed { at, .. } => vec![at],
            Sweep::Swept { from, to, .. } => vec![from,to],
            Sweep::Prism { from, to } => vec![from, to],
            Sweep::Depth { depth } => vec![depth],
            Sweep::Revolve { sweep, .. } => sweep.iter_mut().collect(),
            Sweep::Body | Sweep::Through { .. } | Sweep::Along { .. } | Sweep::Region { .. } => {
                Vec::new()
            }
        }
    }

    /// The revolution axis or through-extent target, for walks that rewrite references.
    pub fn reference_mut(&mut self) -> Option<&mut Ref> {
        match self {
            Sweep::Placed { motion, .. } | Sweep::Swept { motion, .. } => Some(motion),
            Sweep::Revolve { axis, .. } => Some(axis),
            Sweep::Through { body } => Some(body),
            Sweep::Along { guide } => Some(guide),
            Sweep::Region { probe } => Some(probe),
            Sweep::Prism { .. } | Sweep::Depth { .. } | Sweep::Body => None,
        }
    }

    pub fn reference(&self) -> Option<&Ref> {
        match self {
            Sweep::Placed { motion, .. } | Sweep::Swept { motion, .. } => Some(motion),
            Sweep::Revolve { axis, .. } => Some(axis),
            Sweep::Through { body } => Some(body),
            Sweep::Along { guide } => Some(guide),
            Sweep::Region { probe } => Some(probe),
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
    /// `l := line(a, b)` — the point is named, and named somewhere else.
    Ref(Ref),
    /// `line l(hint((0, 0)), …)` — an anonymous point, and where its solve begins.  The
    /// same clause as everywhere else in the language, one level down.
    Hint(KidSeed),
    /// `block := solid(face(a, b, c, -> close), depth: t)` — a private section.
    Face { decl: Box<D>, span: Span },
    /// `tooth := face(root, flank from p to q, tip)` — the stretch of a curve between two points
    /// held on it (`p coincident flank`), a face's edge (§6.8).  Only a face's loop holds one.
    Trim { curve: Ref, from: Ref, to: Ref, span: Span },
}

impl<D> Kid<D> {
    /// The one reference a slot holds, where it holds exactly one — a name, not a trim.
    pub fn as_ref(&self) -> Option<&Ref> {
        match self {
            Kid::Ref(r) => Some(r),
            Kid::Hint(_) | Kid::Face { .. } | Kid::Trim { .. } => None,
        }
    }
    /// Every reference the slot names: a trim names its curve and both points, and a walk
    /// asking what a statement depends on must see all three.
    pub fn refs(&self) -> Vec<&Ref> {
        match self {
            Kid::Ref(r) => vec![r],
            Kid::Trim { curve, from, to, .. } => vec![curve, from, to],
            Kid::Hint(_) | Kid::Face { .. } => Vec::new(),
        }
    }
    pub fn refs_mut(&mut self) -> Vec<&mut Ref> {
        match self {
            Kid::Ref(r) => vec![r],
            Kid::Trim { curve, from, to, .. } => vec![curve, from, to],
            Kid::Hint(_) | Kid::Face { .. } => Vec::new(),
        }
    }
}

/// The seed inside a child slot: an anonymous point's `x` and `y`, or an axis's direction `x`,
/// `y`, `z` (a plane's `u:` and `v:`), carried exactly as `Decl::seed` / `seed_text` /
/// `seed_spans` carry an entity's own scalars, and for the same reasons — a solve splices the
/// numbers and never the words around them.  An unwritten key is an empty span and reads as 0.
#[derive(Clone, Debug, Default)]
pub struct KidSeed {
    pub v: [f64; 3],
    /// As written, where it was written as an expression over the parameters in scope.
    pub text: [Option<String>; 3],
    /// Where each number sits in the source.
    pub spans: [Span; 3],
    /// The whole `hint(…)`, so a writeback that has to add a key can rewrite it.
    pub span: Span,
    /// The slot is an axis's (a plane's `u:` and `v:`), so the seed is its direction, `dir:`.
    pub axis: bool,
}

/// `hint(at: c, bearing: u + phase)` — a place given as geometry: at a point, or at the edge
/// of a circle at a bearing from the page's x-axis; or a step from point `a`:
/// `hint(at: a, toward: b, by: f, turn: θ)` is the fraction `f` (1 if unsaid) of the way to
/// point `b`, turned `θ` about `a`, and `along: l` in place of `toward:` steps by `f` times line
/// `l`'s own run, `p1` to `p2`.  The place keys of the one seed clause (§6.4), read out of it
/// beside the scalars.
/// A place drawn in another view than the seeded point is read where it stands in space and
/// projected into the seeded point's view.
#[derive(Clone, Debug)]
pub struct AtRef {
    pub what: Ref,
    pub bearing: Option<(String, Span)>,
    pub toward: Option<Ref>,
    pub along: Option<Ref>,
    pub by: Option<(String, Span)>,
    pub turn: Option<(String, Span)>,
    /// `hint(at: P, (3, 4))`: the place `(3, 4)` in plane `P`'s own coordinates, read in
    /// space and seen in the seeded point's plane.
    pub x: Option<(String, Span)>,
    pub y: Option<(String, Span)>,
}

impl AtRef {
    /// The place `what` stands, with no step from it.
    pub fn at(what: Ref) -> AtRef {
        AtRef {
            what,
            bearing: None,
            toward: None,
            along: None,
            by: None,
            turn: None,
            x: None,
            y: None,
        }
    }

    /// The texts the place reads numbers from — a bearing, a fraction, a turn, a plane's
    /// coordinates — for the walks that resolve and substitute them.
    pub fn texts_mut(&mut self) -> impl Iterator<Item = &mut (String, Span)> {
        [&mut self.bearing, &mut self.by, &mut self.turn, &mut self.x, &mut self.y]
            .into_iter()
            .flatten()
    }

    pub fn texts(&self) -> impl Iterator<Item = &(String, Span)> {
        [&self.bearing, &self.by, &self.turn, &self.x, &self.y].into_iter().flatten()
    }
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
    /// A whole vector pinned at once: `fix((0, 0)) p`, the point itself, or `fix(dir == (1, 0,
    /// 0)) t`, a vector the entity has.  Its components fill the slots of the members they are
    /// (`Written::assemble`), and it is kept whole here so that what a vector was written as —
    /// how many components — is still known where the entity is (`apply_gauge`).
    Vector { key: Option<Name>, parts: Vec<Arg>, span: Span },
    /// the number, as written — `80`, `x = 7`, `h = w / 2`, `1' 3"`
    Dim(String, Span),
    /// The number beside it is a **bound** (§9.6): `distance(>= 5)`, `distance(<= d)`, or the
    /// low end of `distance(in: (a, b))`, whose high end is `hi`.  `span` is the `>=` or `in:`
    /// written.  Outside a set body a bound is a branch choice: no row, checked on the solution.
    Bound { cmp: Cmp, span: Span, hi: Option<(String, Span)> },
}

/// Which way a bound runs (§9.6).  Closed, every one: a solution may stand on its edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cmp {
    /// `>= d`
    Ge,
    /// `<= d`
    Le,
    /// `in: (a, b)`
    In,
}

impl Cmp {
    /// As written, and as a document stores it.
    pub fn text(self) -> &'static str {
        match self {
            Cmp::Ge => ">=",
            Cmp::Le => "<=",
            Cmp::In => "in",
        }
    }

    pub fn of(text: &str) -> Option<Cmp> {
        [Cmp::Ge, Cmp::Le, Cmp::In].into_iter().find(|c| c.text() == text)
    }
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
    /// The bound the number is, where it is one (§9.6): its direction and an interval's high
    /// end.
    pub fn bound(&self) -> Option<(Cmp, Option<&(String, Span)>)> {
        self.args.iter().find_map(|a| match a {
            OpArg::Bound { cmp, hi, .. } => Some((*cmp, hi.as_ref())),
            _ => None,
        })
    }

    /// Whether it states a bound (§9.6): its number is one, or it is `inside` / `outside` a
    /// plane, lowered to one later — either way no row, and no derivative.
    pub fn states_bound(&self) -> bool {
        self.bound().is_some() || crate::constraints::plane_side_word(&self.word.text).is_some()
    }

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

    /// Every pinned or seeded slot, a vector's components each under the member it is — `x`,
    /// `dir.y` — and where its key was written: what `assemble` fills and a gauge's key reads.
    pub fn slots(&self) -> impl Iterator<Item = (Name, &Arg)> + '_ {
        self.args.iter().flat_map(Self::fills)
    }

    /// The slots one argument fills: a pin or a seed its own, a vector one per member.
    fn fills(a: &OpArg) -> Vec<(Name, &Arg)> {
        match a {
            OpArg::Slot { key, arg } => vec![(key.clone(), arg)],
            OpArg::Vector { key, parts, span } => {
                let span = key.as_ref().map_or(*span, |k| k.span);
                let member = |c: &str| match key {
                    Some(k) => format!("{}.{c}", k.text),
                    None => c.to_string(),
                };
                let names = ["x", "y", "z"].map(member);
                names.into_iter().zip(parts).map(|(text, arg)| (Name { text, span }, arg)).collect()
            }
            _ => Vec::new(),
        }
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
        for (key, _) in self.slots() {
            if !spec.iter().any(|(n, k)| k.is_param() && *n == key.text) {
                let word = &self.word.text;
                let m = format!("`{word}` has no slot `{}` to seed", key.text);
                return Err((key.span, m));
            }
        }
        // and a *selector* naming nothing was dropped in silence, which is the same mistake one
        // layer down (issue #48, item 4): `a distance(80, sied: x) b` settled as a plain distance
        // and the argument went nowhere.  A level's direction stands in its parentheses
        // unlabelled, so `along:` there is a second spelling and refused as one.
        for a in &self.args {
            let OpArg::Named(key, _) = a else { continue };
            if kind == CKind::Level && key.text == "along" {
                let m = "`level` takes its direction in its parentheses: `a level(up) b`, \
                         `a level(std.z) b`".to_string();
                return Err((key.span, m));
            }
            if !spec.iter().any(|(n, _)| *n == key.text) {
                let word = &self.word.text;
                let m = format!("`{word}` takes no `{}`", key.text);
                return Err((key.span, m));
            }
        }
        self.given_once()?;
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
                // a reference given by its slot's name (`at: m`) fills that slot; the operands
                // and the unlabelled references fill the rest in order
                let named = self.args.iter().find_map(|a| match a {
                    OpArg::Named(n, Arg::Ref(r)) if n.text == *name => Some(Arg::Ref(r.clone())),
                    _ => None,
                });
                named.or_else(|| next.next().map(Arg::Ref))
            } else if sk.is_param() {
                let slot = || self.slots().filter(|(key, _)| key.text == *name).map(|(_, arg)| arg);
                // a shared parameter is pinned to an unknown and seeded where the unknown is
                // declared (`param s: Angle hint(330)`): a `hint(t: …)` beside the pin would be a
                // second seed for one unknown, and the contacts sharing it would disagree
                let tie = slot().find(|a| matches!(a, Arg::Tie { .. })).cloned();
                if tie.is_some() {
                    if let Some(key) = self.args.iter().find_map(|a| match a {
                        OpArg::Slot { key, arg: Arg::Seed { pinned: false, .. } }
                            if key.text == *name => Some(key),
                        _ => None,
                    }) {
                        let m = format!(
                            "`{}` is pinned to an unknown, which is seeded where it is declared",
                            key.text
                        );
                        return Err((key.span, m));
                    }
                }
                tie.or_else(|| slot().next().cloned())
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
        if let Some(w) = kind.word_slot() {
            Self::direction(w, &mut out);
        }
        Ok(out)
    }

    /// **A slot is given once** (issue #112): assembled, a slot takes the first argument naming
    /// it, so a second — `fix((3, 4), (1, 1)) a`, `fix((3, 4), x == 1) a`, two pins of one
    /// contact — went nowhere and said nothing.  Refused at the second, saying which.  A pin to
    /// an unknown beside a seed for it has a message of its own (`assemble`'s tie).
    fn given_once(&self) -> Result<(), (Span, String)> {
        let seed = |x: &Arg| {
            matches!(x, Arg::Seed { pinned: false, .. } | Arg::SeedExpr { pinned: false, .. })
        };
        let tie = |x: &Arg| matches!(x, Arg::Tie { .. });
        // each slot given so far, and whether by a place written whole (`(3, 4)`)
        let mut said: Vec<(String, &Arg, bool)> = Vec::new();
        for a in &self.args {
            let whole = matches!(a, OpArg::Vector { key: None, .. });
            for (key, arg) in Self::fills(a) {
                let before = said.iter().find(|(n, b, _)| {
                    *n == key.text && !(tie(arg) && seed(b) || seed(arg) && tie(b))
                });
                if let Some(&(_, _, was_whole)) = before {
                    let word = &self.word.text;
                    let m = if whole && was_whole {
                        format!("`{word}` is given its place twice: a point is one vector")
                    } else {
                        format!("`{}` is given twice: a number is said once", key.text)
                    };
                    return Err((key.span, m));
                }
                said.push((key.text, arg, whole));
            }
        }
        Ok(())
    }

    /// **An ordinate's direction, where it was written** (`docs/ordinate-plan.md`): `along:` an
    /// ordinate is measured on and the parentheses of `level` hold a word of
    /// `constraints::ALONG` or a reference.  The word goes to the kind's word slot — the core
    /// infers the axis it names — and a reference to its direction slot.
    fn direction(w: usize, out: &mut [Option<Arg>]) {
        let t = 2;
        match out[w].take() {
            Some(Arg::Ref(r)) => out[t] = Some(Arg::Ref(r)),
            other => out[w] = other,
        }
        if let Some(Arg::Ref(r)) = &out[t] {
            if let Some(v) = r.direction_word().map(str::to_string) {
                out[t] = None;
                out[w] = Some(Arg::Word(v));
            }
        }
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
    /// The defined word this relation was written with, where it was (§9.9): the flattener
    /// replaces the statement's form by the word's body and keeps the word here.
    pub word: Option<Worded>,
    /// Set by the flattener on a set's body row stated as its derivative (§6.21).
    pub along: Option<Along>,
}

impl Relation {
    /// A relation built rather than written: its form and nothing else — no placement, class,
    /// claim, word or derivative.
    pub fn of(form: RelationForm) -> Relation {
        Relation {
            form,
            place: None,
            place_span: Span::default(),
            claim: false,
            class: Classes::default(),
            class_span: Span::default(),
            word: None,
            along: None,
        }
    }
}

/// Parsed operators and generated registry calls are mutually exclusive.
#[derive(Clone, Debug)]
pub enum RelationForm {
    Written(Written),
    /// `bound`: the number is a bound (§9.6), its direction and an interval's high end.
    Canonical { kind: CKind, args: Vec<Option<Arg>>, bound: Option<(Cmp, Option<Arg>)> },
}

impl RelationForm {
    /// The bound its number is, where it is one (§9.6), in either form: an interval's high end
    /// as the text written, or the number lifted.
    pub fn bound(&self) -> Option<(Cmp, Option<Arg>)> {
        match self {
            Self::Written(w) => w.bound().map(|(cmp, hi)| {
                (cmp, hi.map(|(text, span)| Arg::Dim { text: text.clone(), span: *span }))
            }),
            Self::Canonical { bound, .. } => bound.clone(),
        }
    }

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
    /// A pin to a name nothing in scope defines — `t == s` — which makes the slot's unknown a
    /// **shared contact parameter**: every contact pinned to the same name owns the one unknown,
    /// as every dimension reading a free variable shares it (issue #70, part 2).  Made by the
    /// flattener from a `SeedExpr`, `name` absolute (`leg.s` inside an instance); `seed` is the
    /// statement's own `hint(t: …)`, put beside it by `Written::assemble`.
    Tie {
        name: String,
        seed: Option<f64>,
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
    /// The direction word this reference is, where it is a bare one of `constraints::ALONG` —
    /// `level(up)`'s `up`, which names a direction and no entity.
    pub fn direction_word(&self) -> Option<&str> {
        let w = self.root.text.as_str();
        (self.path.is_empty() && crate::constraints::Toward::of(w).is_some()).then_some(w)
    }

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
