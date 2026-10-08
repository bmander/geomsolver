//! Expanded operations consumed by geometry elaboration.
//! Source editing metadata stays in `syntax`; IDs, spans and instance paths cross this boundary.

use crate::constraints::CKind;
use crate::model::EntKind;
use crate::style::Classes;
use crate::syntax::{
    self, Arg, AtRef, CurveSpec, DeclName, Membership, Name, Ref, RelationForm,
    Span, StmtId, Sweep, Written,
};

/// The source construct and copy that led to an expanded operation.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PathStep {
    Instance(StmtId),
    Copy { block: StmtId, index: u32 },
}

#[derive(Clone, Debug)]
pub struct Statement {
    pub id: StmtId,
    pub span: Span,
    pub path: Vec<PathStep>,
    pub kind: Operation,
    /// Of a `ring`'s turned copy (k ≥ 1, §12.4): its declarations are built — a copy is an
    /// entity — and its relations, holds and claims are not, being the representative's turned.
    pub turned: bool,
}

#[derive(Clone, Debug)]
pub enum Operation {
    Decl(Box<Decl>),
    Chain(syntax::NamedChain),
    Relation(Relation),
    Branch(syntax::Branch),
    Style(syntax::StyleRule),
    Unit(Name),
    SolidRel(syntax::SolidRel),
    ClaimOver(ClaimOver),
    Derived(syntax::DerivedDecl),
    /// `k minimizes …` (#121): its curve, resolved, and its integrands as text over
    /// `p.x`, `p.y`, `t.x`, `t.y`.
    Minimize(syntax::Minimize),
}

#[derive(Clone, Debug)]
pub struct Decl {
    pub roles: crate::semantics::GeometryRoles,
    pub kind: EntKind,
    pub name: DeclName,
    pub children: Vec<Vec<Kid>>,
    pub seed: Vec<f64>,
    pub seed_text: Vec<Option<String>>,
    pub seed_spans: Vec<Span>,
    pub unseeded: bool,
    pub seed_explicit: Vec<bool>,
    pub closed: bool,
    pub knots: Option<Vec<f64>>,
    /// Settled by expansion: every text worked out.
    pub weights: Option<Vec<f64>>,
    pub curve: Option<CurveSpec>,
    pub computed: Option<[(String, Span); 2]>,
    pub class: Classes,
    pub seed_at: Option<AtRef>,
    pub seed_names: Vec<(String, String)>,
    pub sweep: Option<Sweep>,
    pub motion: Option<syntax::MotionSpec>,
    pub angular_span: Option<syntax::AngularSpan>,
    pub membership: Membership,
}

/// The same operands as syntax, with inline faces lowered to declarations of this stage.
pub type Kid = syntax::Kid<Decl>;

impl From<syntax::Decl> for Decl {
    fn from(d: syntax::Decl) -> Self {
        Self {
            roles: d.annotations.roles,
            kind: d.kind,
            name: d.name,
            children: d.children.into_iter().map(|g| {
                g.into_iter().map(|k| match k {
                    syntax::Kid::Ref(r) => Kid::Ref(r),
                    syntax::Kid::Hint(s) => Kid::Hint(s),
                    syntax::Kid::Face { decl, span } =>
                        Kid::Face { decl: Box::new((*decl).into()), span },
                    syntax::Kid::Trim { curve, from, to, span } => Kid::Trim { curve, from, to, span },
                }).collect()
            }).collect(),
            seed_text: d.seed_text,
            unseeded: d.seed_at.is_none() && d.hint_span.is_some_and(|s| s.is_empty()),
            seed_explicit: (0..d.seed.len())
                .map(|i| {
                    d.hint_span.is_none() || d.seed_spans.get(i).is_some_and(|s| !s.is_empty())
                })
                .collect(),
            seed_spans: d.seed_spans,
            closed: d.close.is_some(),
            seed: d.seed,
            knots: d.knots,
            weights: d.weights.map(|w| w.into_iter().map(|w| w.value).collect()),
            curve: d.curve,
            computed: d.computed,
            class: d.class,
            seed_at: d.seed_at,
            seed_names: d.seed_names,
            sweep: d.sweep,
            motion: d.motion,
            angular_span: d.angular_span,
            membership: d.membership,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Relation {
    pub form: RelationForm,
    pub place: Option<(f64, f64)>,
    pub claim: bool,
    pub class: Classes,
    /// The defined word the statement was written with (§9.9) — see `syntax::Relation::word`.
    pub word: Option<syntax::Worded>,
    /// A set's body row stated as its derivative (§6.21) — see `syntax::Relation::along`.
    pub along: Option<syntax::Along>,
}

/// An operator selected against the declared entity kinds, before argument validation.
pub(crate) struct ResolvedRelation<'a> {
    pub kind: CKind,
    pub args: Vec<Option<Arg>>,
    pub written: Option<&'a Written>,
    pub claim: bool,
    pub class: &'a Classes,
}

#[derive(Clone, Debug)]
pub struct ClaimOver {
    pub formal: Ref,
    pub from: Arg,
    pub to: Arg,
    pub body: Vec<Statement>,
    pub span: Span,
}

impl Statement {
    pub(crate) fn lower(
        st: syntax::Stmt,
        path: Vec<PathStep>,
    ) -> Result<Self, crate::program::Diag> {
        use syntax::StmtKind as S;
        let kind = match st.kind {
            S::Chain(c) => Operation::Chain(c),
            S::Decl(d) => Operation::Decl(Box::new(d.into())),
            S::Relation(r) => Operation::Relation(Relation {
                form: r.form, place: r.place, claim: r.claim, class: r.class, word: r.word,
                along: r.along,
            }),
            S::Branch(b) => Operation::Branch(b),
            S::Style(s) => Operation::Style(s),
            S::Unit(n) => Operation::Unit(n),
            S::SolidRel(r) => Operation::SolidRel(r),
            S::Derived(d) => Operation::Derived(d),
            S::Minimize(m) => Operation::Minimize(m),
            S::ClaimOver(c) => Operation::ClaimOver(ClaimOver {
                formal: c.formal, from: c.from, to: c.to, span: c.span,
                body: c.body.into_iter().map(|s| Self::lower(s, path.clone())).collect::<Result<_, _>>()?,
            }),
            S::Param(_) | S::Group(_) | S::Instance(_) | S::Block(_) | S::Set(_) => return Err(crate::program::Diag {
                code: crate::program::Code::E103, span: st.span, stmt: Some(st.id),
                message: "parameters, groups, sets, component instances and repetition are not supported inside a swept claim".into(),
            }),
        };
        Ok(Self { id: st.id, span: st.span, path, kind, turned: false })
    }
}
