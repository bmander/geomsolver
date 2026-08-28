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
use crate::syntax::{
    entity_name, line_col, num, Arg, Decl, Gauge, Name, Orient, Program, Ref, Relation, Seg, Span,
    Stmt, StmtId, StmtKind,
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
    /// type mismatch within an alias class
    E040,
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
}

impl Code {
    pub fn as_str(self) -> &'static str {
        match self {
            Code::E001 => "E001",
            Code::E040 => "E040",
            Code::E100 => "E100",
            Code::E101 => "E101",
            Code::E102 => "E102",
            Code::E103 => "E103",
            Code::E104 => "E104",
            Code::E105 => "E105",
            Code::E106 => "E106",
            Code::W110 => "W110",
            Code::W111 => "W111",
        }
    }

    pub fn severity(self) -> Severity {
        match self {
            Code::W110 | Code::W111 => Severity::Warning,
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
    /// Every name an entity was declared or aliased under.  A `Vec` from the start, because a
    /// port puts several names on one entity and costs no residual for doing it.
    pub names: BTreeMap<EntRef, Vec<String>>,
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

    fn bind(&mut self, name: &str, e: EntRef) {
        self.by_name.insert(name.to_string(), e);
        self.names.entry(e).or_default().push(name.to_string());
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
        let Some(prog) = reparse(text) else { return false };
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
        let Some(prog) = reparse(text) else { return false };
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
                        self.map.bind(&d.name.text, r);
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
/// alias class *is* how a port costs nothing, so aliasing is a property of this pass rather than
/// a later change to the model.
#[derive(Default)]
struct Resolver {
    of: BTreeMap<String, EntRef>,
    declared_at: BTreeMap<String, Span>,
}

impl Resolver {
    fn lookup(&self, r: &Ref) -> Option<EntRef> {
        self.of.get(&r.root.text).copied()
    }
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
fn reparse(text: &str) -> Option<Program> {
    let (prog, errs) = crate::syntax::parse(text);
    errs.is_empty().then_some(prog)
}

/// Where every statement in a program sits, blocks included — a statement inside one is still
/// reached by its id.
fn spans(prog: &Program) -> BTreeMap<StmtId, Span> {
    fn stamp(st: &Stmt, out: &mut BTreeMap<StmtId, Span>) {
        out.insert(st.id, st.span);
        if let StmtKind::Block(b) = &st.kind {
            for inner in b.body.iter() {
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

pub fn elaborate(p: &Program) -> Elaborated {
    let mut diags: Vec<Diag> = Vec::new();
    let mut map = SourceMap::default();
    let mut sk = Sketch::new();

    // -- phase 1: names, in one pre-pass.  Indices come from declaration order within a kind,
    // which is `primitives()` order, which is the order phase 2 builds in.
    // curve families first: an instance names one, and the tapes have to exist before any
    // contact with them is compiled
    for f in &p.curves {
        match compile_family(f) {
            Ok(d) => sk.curve_defs.push(d),
            Err((span, message)) => {
                diags.push(Diag { code: Code::E103, span, stmt: None, message })
            }
        }
    }
    let expansion = crate::flatten::expand(p);
    for (span, message) in &expansion.errors {
        diags.push(Diag {
            code: if message.starts_with("no such") { Code::E101 } else { Code::E103 },
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
    for st in &body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        if let Some(&was) = res.declared_at.get(&d.name.text) {
            diags.push(Diag {
                code: Code::E001,
                span: d.name.span,
                stmt: Some(st.id),
                message: format!(
                    "`{}` is declared twice; the first is at line {}",
                    d.name.text,
                    line_col(p.text(), was.lo).0
                ),
            });
            skip.insert(st.id);
            continue; // the second, so every later reference still resolves to the first
        }
        let n = count.entry(d.kind).or_insert(0);
        res.of.insert(d.name.text.clone(), EntRef::new(d.kind, *n as usize));
        res.declared_at.insert(d.name.text.clone(), d.name.span);
        *n += 1;
    }

    // -- phase 2: geometry, per kind in `primitives()` order.  The same walk `io::from_json`
    // makes, through the same constructors, so the two produce the same parameter vector.
    let mut built: BTreeMap<EntRef, bool> = BTreeMap::new();
    // `primitives()` order, and **curves last**: a curve is written over other entities, so
    // every kind it may name has to exist before it does.  The same reason `io::graft` grafts
    // them last.
    for kind in [
        EntKind::Point,
        EntKind::Line,
        EntKind::Circle,
        EntKind::Arc,
        EntKind::Spline,
        EntKind::Ellipse,
        EntKind::Frame,
        EntKind::Curve,
    ] {
        for st in &body {
            let StmtKind::Decl(d) = &st.kind else { continue };
            if d.kind != kind || skip.contains(&st.id) {
                continue;
            }
            match build(&mut sk, &res, d, st, &mut diags) {
                Some(e) => {
                    built.insert(e, true);
                    map.bind(&d.name.text, e);
                    map.record(st, Made::Ent(e));
                }
                None => {
                    // a declaration that could not be built leaves its name unbound, so every
                    // reference to it is reported where the reference is
                    res.of.remove(&d.name.text);
                }
            }
        }
    }

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

    // -- phase 4: gauges
    for st in &body {
        let StmtKind::Gauge(g) = &st.kind else { continue };
        gauge(&mut sk, &res, g, st, &mut diags);
    }

    // -- phase 5: every expression against the whole document, once.  Per-statement evaluation
    // would be quadratic in the expression count and would make a dimension whose definition is
    // further down the file briefly a free variable — allocating an unknown the next pass retires.
    for item in expr::evaluate(&mut sk) {
        let span = map.site_of_constraint(item.id).map(|s| s.span).unwrap_or_default();
        let stmt = map.site_of_constraint(item.id).map(|s| s.stmt);
        if let Some(err) = &item.error {
            diags.push(Diag {
                code: Code::W110,
                span,
                stmt,
                message: format!("`{}`: {err} — the last number stands", item.text),
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

    // -- phase 6: recorded root choices, by name
    for st in &body {
        let StmtKind::Orient(o) = &st.kind else { continue };
        orient(&mut sk, &res, o, st, &mut diags);
    }

    Elaborated { sketch: sk, map, diags, program: p.clone(), taken: false }
}

/// A curve family, compiled.
///
/// The variable table is the parameter, then every coordinate its entity formals contribute *in
/// `entity_params` order* (`EntKind::scalar_names`), then the numbers it takes.  That order is
/// the kernel's column order, which is what makes a tape's gradient a row of the Jacobian.
fn compile_family(f: &crate::syntax::CurveFamily) -> Result<crate::model::CurveDef, (Span, String)> {
    use crate::syntax::{FamilyBody, Ty};
    let mut vars = vec![f.param.text.clone()];
    let mut formals = Vec::new();
    let mut values = Vec::new();
    for fo in &f.formals {
        match fo.ty {
            Ty::Ent(k) => {
                let names = k.scalar_names(&fo.name.text).ok_or_else(|| {
                    (
                        fo.span,
                        format!(
                            "a curve cannot be written over a {}: it has no fixed number of                              coordinates",
                            k.as_str()
                        ),
                    )
                })?;
                vars.extend(names);
                formals.push((fo.name.text.clone(), k));
            }
            _ => values.push(fo.name.text.clone()),
        }
    }
    vars.extend(values.iter().cloned());
    let body = match &f.body {
        FamilyBody::Exprs { x, y, xspan, yspan } => {
            let tape = |text: &str, span: Span| -> Result<crate::tape::Tape, (Span, String)> {
                let ast = crate::expr::parse(text).map_err(|e| (span, e))?;
                crate::tape::Tape::compile(&ast.body, &vars).map_err(|e| (span, e))
            };
            crate::model::CurveBody::Exprs { x: tape(x, *xspan)?, y: tape(y, *yspan)? }
        }
        FamilyBody::Trace { point, home, body } => crate::model::CurveBody::Trace(
            compile_trace(f, point, home.as_ref(), body, &vars, &formals, values.len())?,
        ),
    };
    let num = |t: &str| crate::expr::literal(t).unwrap_or(0.0);
    let domain = match &f.domain {
        Some((a, b)) => (num(a), num(b)),
        None => (0.0, 1.0),
    };
    Ok(crate::model::CurveDef {
        name: f.name.text.clone(),
        formals,
        values,
        param: f.param.text.clone(),
        vars,
        body,
        domain,
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
fn compile_trace(
    f: &crate::syntax::CurveFamily,
    point: &crate::syntax::Name,
    home: Option<&(String, Span)>,
    body: &[Stmt],
    vars: &[String],
    formals: &[(String, EntKind)],
    n_values: usize,
) -> Result<crate::locus::Locus, (Span, String)> {
    use crate::locus::{Locus, Pred, Row};
    use crate::tape::Tape;
    let mut sk = Sketch::new();
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
            EntKind::Frame => {
                let o = sk.point(0.0, 0.0, false, name);
                let t = sk.point(1.0, 0.0, false, name);
                EntRef::frame(sk.frame(o, t, name))
            }
            other => {
                return Err((
                    f.span,
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
        let ast = crate::expr::parse(text).map_err(|e| (span, e))?;
        Tape::compile(&ast.body, vars).map_err(|e| (span, e))
    };
    let constant = |v: f64| -> Tape {
        Tape::compile(&crate::expr::Ast::Num(v), vars).expect("a number always compiles")
    };

    // -- pass 1: declarations, so a statement may read a point declared after it --------
    let mut n_q = 0usize;
    let mut seeds: Vec<Tape> = Vec::new();
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
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
                .ok_or((st.span, format!("`{}` needs its points named", d.name.text)))?;
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
                let e = EntRef::point(sk.point(0.0, 0.0, false, &d.name.text));
                seeds.push(sx);
                seeds.push(sy);
                for p in sk.entity_params(e) {
                    slot.insert(p, n_outer + n_q);
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
                let e = EntRef::circle(sk.circle(c.i(), 0.0, &d.name.text));
                seeds.push(sr);
                slot.insert(sk.circles[e.i()].radius, n_outer + n_q);
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
        if scope.insert(d.name.text.clone(), e).is_some() {
            return Err((st.span, format!("`{}` is declared twice", d.name.text)));
        }
    }

    // -- pass 2: constraints and orientations, lowered to rows and predicates ----------
    let mut w: Vec<Tape> = Vec::new();
    let mut rows: Vec<Row> = Vec::new();
    let mut preds: Vec<Pred> = Vec::new();
    for st in body {
        let r = match &st.kind {
            StmtKind::Decl(_) => continue,
            // `ccw(a, b, x)` — no residual: it selects among the discrete solution components,
            // which is what a branch is, and it is how a block says one *as a fact* rather than
            // as a place to start looking
            StmtKind::Orient(o) => {
                if o.pts.len() != 3 {
                    return Err((st.span, "an orientation names three points".to_string()));
                }
                let mut cols = [0u32; 6];
                for (k, rf) in o.pts.iter().enumerate() {
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
                        o.pts[2].span,
                        "the third point must be one the block places".to_string(),
                    ));
                }
                preds.push(Pred { ccw: o.ccw, cols });
                continue;
            }
            StmtKind::Relation(r) => r,
            _ => {
                return Err((
                    st.span,
                    "a trace block holds declarations and constraints only".to_string(),
                ))
            }
        };
        let spec = r.kind.spec();
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
    let traced = match scope.get(&point.text) {
        Some(e) if e.kind == EntKind::Point => {
            let p = sk.point_params(e.i())[0];
            match slot.get(&p) {
                Some(&s) if s >= n_outer => s - n_outer,
                _ => {
                    return Err((
                        point.span,
                        format!("`{}` must be a point the block declares", point.text),
                    ))
                }
            }
        }
        _ => {
            return Err((
                point.span,
                format!("`{}` must be a point the block declares", point.text),
            ))
        }
    };
    let home_tape = match home {
        Some((text, sp)) => Some(tape(text, *sp)?),
        None => None,
    };
    Locus::new(n_outer, n_theta, n_q, traced, w, seeds, rows, preds, home_tape)
        .map_err(|m| (f.span, m))
}

/// A seed named geometrically, compiled to the tapes a written pair would be: the place a point
/// already names, or **the point at the edge of a circle** at a bearing from the page's x-axis —
/// `at c bearing (u + phase)`, which is what that place is called in this language rather than
/// the trigonometry it comes to.
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
            let beta = crate::expr::parse(text).map_err(|m| (*bsp, m))?.body;
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
            "where on the edge?  `at c bearing (…)` says the bearing".to_string(),
        )),
        (k, _) => Err((a.what.span, format!("a seed cannot be at a {}", k.as_str()))),
    }
}

fn build(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<EntRef> {
    // a curve is the one kind whose arguments need not be points, so it is built before the
    // walk that insists they are
    if d.kind == EntKind::Curve {
        return build_curve(sk, res, d, st, diags);
    }
    // a geometric seed reads geometry that a trace block's variable table names; out here a
    // drawing's seed is a number a solve writes back, which a place named by reference is not
    if d.seed_at.is_some() {
        diags.push(Diag {
            code: Code::E103,
            span: st.span,
            stmt: Some(st.id),
            message: "a seed named geometrically (`at c bearing (…)`) lives in a trace block"
                .to_string(),
        });
    }
    // every child a declaration names, flattened in field order and checked to be a Point —
    // which every other child of every other kind is, and which is what an alias class must
    // agree about
    let mut kids: Vec<usize> = Vec::new();
    for group in &d.children {
        for r in group {
            let Some(e) = res.lookup(r) else {
                diags.push(Diag {
                    code: Code::E101,
                    span: r.span,
                    stmt: Some(st.id),
                    message: format!("no such entity: `{}`", r.root.text),
                });
                return None;
            };
            let e = match follow(sk, e, &r.path) {
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
                    kids.len()
                ),
            });
            return None;
        }
    }
    let seed = |i: usize| d.seed.get(i).copied().unwrap_or(0.0);
    let idx = match d.kind {
        EntKind::Point => sk.point(seed(0), seed(1), false, &d.name.text),
        EntKind::Line => sk.line(kids[0], kids[1]),
        EntKind::Circle => sk.circle(kids[0], seed(0), &d.name.text),
        EntKind::Arc => {
            // `arc` adds the two intrinsic `PointOnCircle`s here and nowhere else, and computes a
            // radius from the geometry that the declared seed then replaces
            let ai = sk.arc(kids[0], kids[1], kids[2], &d.name.text);
            let rp = sk.arcs[ai].radius as usize;
            sk.params[rp].value = seed(0);
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
        EntKind::Ellipse => sk.ellipse(kids[0], kids[1], seed(0), &d.name.text),
        EntKind::Frame => {
            // `frame` adds the two intrinsics here and nowhere else, and computes a rotor from
            // the chord that a declared seed then replaces — except (0, 0), which is no rotor
            // at all and is what an unwritten seed reads as
            let fi = sk.frame(kids[0], kids[1], &d.name.text);
            let (c, s) = (seed(0), seed(1));
            if c != 0.0 || s != 0.0 {
                let f = &sk.frames[fi];
                let (cp, sp) = (f.c as usize, f.s as usize);
                sk.params[cp].value = c;
                sk.params[sp].value = s;
            }
            fi
        }
        EntKind::Curve => unreachable!("a curve is built before this walk"),
    };
    let e = EntRef::new(d.kind, idx);
    set_construction(sk, e, d.construction);
    Some(e)
}


/// A curve instance: a family, the entities it is written over, and the numbers it takes.
fn build_curve(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<EntRef> {
let Some(fam) = d.def.as_ref() else {
        diags.push(Diag {
            code: Code::E103,
            span: st.span,
            stmt: Some(st.id),
            message: "a curve is drawn as `curve name = family(args)`".to_string(),
        });
        return None;
    };
    let Some(di) = sk.curve_defs.iter().position(|x| x.name == fam.text) else {
        diags.push(Diag {
            code: Code::E101,
            span: fam.span,
            stmt: Some(st.id),
            message: format!("no curve family named `{}`", fam.text),
        });
        return None;
    };
    let want = sk.curve_defs[di].formals.clone();
    let args: Vec<EntRef> = d
        .children
        .first()
        .map(|g| g.iter().filter_map(|r| res.lookup(r)).collect())
        .unwrap_or_default();
    if args.len() != want.len() {
        diags.push(Diag {
            code: Code::E103,
            span: st.span,
            stmt: Some(st.id),
            message: format!(
                "`{}` is written over {} entit(ies), and {} were given",
                fam.text,
                want.len(),
                args.len()
            ),
        });
        return None;
    }
    for (a, (fname, k)) in args.iter().zip(&want) {
        if a.kind != *k {
            diags.push(Diag {
                code: Code::E040,
                span: st.span,
                stmt: Some(st.id),
                message: format!(
                    "`{fname}` is a {}, and a {} was given",
                    k.as_str(),
                    a.kind.as_str()
                ),
            });
            return None;
        }
    }
    // the numbers it was given, in the family's own order
    let names = sk.curve_defs[di].values.clone();
    let values: Vec<f64> = names
        .iter()
        .map(|n| {
            d.values
                .iter()
                .find(|(l, _)| &l.text == n)
                .and_then(|(_, t)| crate::expr::literal(t))
                .unwrap_or(0.0)
        })
        .collect();
    let domain = match &d.domain {
        Some((a, b)) => (
            crate::expr::literal(a).unwrap_or(0.0),
            crate::expr::literal(b).unwrap_or(1.0),
        ),
        None => sk.curve_defs[di].domain,
    };
    sk.curves.push(crate::model::CurveE {
        def: di as u32,
        args,
        values,
        domain,
        construction: d.construction,
    });
    Some(EntRef::new(EntKind::Curve, sk.curves.len() - 1))
}

fn set_construction(sk: &mut Sketch, e: EntRef, on: bool) {
    match e.kind {
        EntKind::Line => sk.lines[e.i()].construction = on,
        EntKind::Curve => sk.curves[e.i()].construction = on,
        EntKind::Circle => sk.circles[e.i()].construction = on,
        EntKind::Arc => sk.arcs[e.i()].construction = on,
        EntKind::Spline => sk.splines[e.i()].construction = on,
        EntKind::Ellipse => sk.ellipses[e.i()].construction = on,
        EntKind::Frame => sk.frames[e.i()].construction = on,
        EntKind::Point => {}
    }
}

fn constrain(
    sk: &mut Sketch,
    res: &Resolver,
    r: &Relation,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<u32> {
    // a drafting word the parser could not settle: `a equal b` between two *names*, where what
    // `equal` means is the kinds it stands between and a name's kind is not known until here.
    // Resolved before the spec is read, since the spec is what the arguments are checked against
    // — an `EqualLength` placeholder would reject two arcs as "not a line" before ever asking.
    let ckind = match &r.poly {
        None => r.kind,
        Some(word) => {
            let ent = |i: usize| match r.args.get(i).and_then(|a| a.as_ref()) {
                Some(Arg::Ref(re)) => res.lookup(re).map(|e| e.kind),
                _ => None,
            };
            match (ent(0), ent(1)) {
                (Some(a), Some(b)) => match crate::syntax::equal_kind(a, b) {
                    Some(k) => k,
                    None => {
                        diags.push(Diag {
                            code: Code::E040,
                            span: word.span,
                            stmt: Some(st.id),
                            message: format!(
                                "`{}` does not relate a {} to a {}",
                                word.text,
                                a.as_str(),
                                b.as_str()
                            ),
                        });
                        return None;
                    }
                },
                // a name that resolves to nothing: the reference is reported below, on the
                // argument itself, where the message can say which name it was
                _ => r.kind,
            }
        }
    };
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
    // lives, shared with the document reader and the bindings' constraint records
    io::seed_omitted(sk, ckind, &mut args, |i| left_out[i]);
    let mut c = Constraint::new(ckind, args);
    c.claim = r.claim;
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
                    expr::parse(text).map_err(|e| e.to_string())?;
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

fn gauge(sk: &mut Sketch, res: &Resolver, g: &Gauge, st: &Stmt, diags: &mut Vec<Diag>) {
    let (r, whole) = match g {
        Gauge::Ground(r) => (r, true),
        Gauge::Fix(r) => (r, false),
    };
    let Some(e) = res.lookup(r) else {
        diags.push(Diag {
            code: Code::E101,
            span: r.span,
            stmt: Some(st.id),
            message: format!("no such entity: `{}`", r.root.text),
        });
        return;
    };
    let e = if whole { follow(sk, e, &r.path).unwrap_or(e) } else { e };
    if whole {
        if e.kind != EntKind::Point {
            diags.push(Diag {
                code: Code::E105,
                span: st.span,
                stmt: Some(st.id),
                message: "ground pins a point; a scalar is pinned with fix".to_string(),
            });
            return;
        }
        sk.fix_point(e.i(), true);
        return;
    }
    // `fix(c0.r)`: the entity's own scalar, named by the field it is.  The document stores one
    // flag per scalar and nothing finer, so neither does this.
    let field = match r.path.first() {
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
        _ => diags.push(Diag {
            code: Code::E105,
            span: st.span,
            stmt: Some(st.id),
            message: if scalars.is_empty() {
                format!("a {} has no number of its own to fix", e.kind.as_str())
            } else {
                format!("a {} has {}, not `{field}`", e.kind.as_str(), scalars.join(" and "))
            },
        }),
    }
}

fn orient(sk: &mut Sketch, res: &Resolver, o: &Orient, st: &Stmt, diags: &mut Vec<Diag>) {
    if let Some((key, v)) = &o.raw {
        sk.branches.insert(key.clone(), *v);
        return;
    }
    let mut pts = [0usize; 3];
    if o.pts.len() != 3 {
        diags.push(Diag {
            code: Code::E103,
            span: st.span,
            stmt: Some(st.id),
            message: "an orientation names three points".to_string(),
        });
        return;
    }
    for (i, r) in o.pts.iter().enumerate() {
        match res.lookup(r).and_then(|e| follow(sk, e, &r.path).ok()) {
            Some(e) if e.kind == EntKind::Point => pts[i] = e.i(),
            _ => {
                diags.push(Diag {
                    code: Code::E101,
                    span: r.span,
                    stmt: Some(st.id),
                    message: format!("no such point: `{}`", r.root.text),
                });
                return;
            }
        }
    }
    sk.branches.insert(decompose::branch_key(pts), if o.ccw { 1 } else { -1 });
}

/* -- the lift ---------------------------------------------------------------------- */

/// The canonical program for a sketch.
///
/// Every `.json` document ever saved becomes a program through this, which is the whole of the
/// migration — and, while the parser is still being written, the whole of the bootstrap: a panel
/// can show a program before anything can read one back.
pub fn to_program(sk: &Sketch) -> Program {
    let mut p = Program::new();
    for e in sk.primitives() {
        p.push(StmtKind::Decl(lift_decl(sk, e)));
    }
    for c in sk.user_constraints() {
        p.push(StmtKind::Relation(lift_relation(sk, c)));
    }
    for i in 0..sk.points.len() {
        if sk.point_fixed(i) {
            p.push(StmtKind::Gauge(Gauge::Ground(Ref::new(entity_name(EntRef::point(i))))));
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
                p.push(StmtKind::Gauge(Gauge::Fix(Ref::field(entity_name(e), f))));
            }
        }
    }
    for (key, &v) in &sk.branches {
        p.push(StmtKind::Orient(match decompose::branch_key_points(key) {
            Some(t) => Orient {
                ccw: v >= 0,
                pts: t.iter().map(|&i| Ref::new(entity_name(EntRef::point(i)))).collect(),
                raw: None,
            },
            // a key that is not a triple of points has no name to travel under; it is kept
            // verbatim so a document never silently loses one
            None => Orient { ccw: true, pts: Vec::new(), raw: Some((key.clone(), v)) },
        }));
    }
    crate::syntax::render(&mut p);
    p
}

pub(crate) fn lift_decl(sk: &Sketch, e: EntRef) -> Decl {
    let kids = sk.children(e);
    let mut children: Vec<Vec<Ref>> = Vec::new();
    let mut taken = 0usize;
    for (_, field) in e.kind.fields() {
        match field {
            Field::Child => {
                children.push(kids.get(taken).map(|&k| vec![Ref::new(entity_name(k))]).unwrap_or_default());
                taken += 1;
            }
            Field::List => {
                children.push(kids[taken..].iter().map(|&k| Ref::new(entity_name(k))).collect());
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
        name: Name::new(entity_name(e)),
        children,
        seed_text: vec![None; seed.len()],
        seed_spans: vec![Span::default(); seed.len()],
        seed,
        def: (e.kind == EntKind::Curve)
            .then(|| Name::new(sk.curve_defs[sk.curves[e.i()].def as usize].name.clone())),
        values: Vec::new(),
        domain: None,
        knots,
        construction: construction_of(sk, e),
        seed_at: None,
    }
}

pub(crate) fn construction_of(sk: &Sketch, e: EntRef) -> bool {
    match e.kind {
        EntKind::Line => sk.lines[e.i()].construction,
        EntKind::Curve => sk.curves[e.i()].construction,
        EntKind::Circle => sk.circles[e.i()].construction,
        EntKind::Arc => sk.arcs[e.i()].construction,
        EntKind::Spline => sk.splines[e.i()].construction,
        EntKind::Ellipse => sk.ellipses[e.i()].construction,
        EntKind::Frame => sk.frames[e.i()].construction,
        EntKind::Point => false,
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
