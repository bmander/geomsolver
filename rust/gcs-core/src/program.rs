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

use crate::constraints::{Arg as CArg, Constraint, SpecKind};
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
    /// The text every span in here indexes.  Carried rather than borrowed, because a span without
    /// the text it cuts is a pair of numbers about nothing — and because the one caller that most
    /// needs both is across an ABI, where a borrow cannot follow.
    pub text: String,
    /// Whether the sketch has been moved out.  There was only ever one, and a second taker would
    /// get an empty sketch that looked like a real one.
    pub taken: bool,
}

impl Elaborated {
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

    Elaborated { sketch: sk, map, diags, text: p.text().to_string(), taken: false }
}

/// A curve family, compiled.
///
/// The variable table is the parameter, then every coordinate its entity formals contribute *in
/// `entity_params` order* (`EntKind::scalar_names`), then the numbers it takes.  That order is
/// the kernel's column order, which is what makes a tape's gradient a row of the Jacobian.
fn compile_family(f: &crate::syntax::CurveFamily) -> Result<crate::model::CurveDef, (Span, String)> {
    use crate::syntax::Ty;
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
    let tape = |text: &str, span: Span| -> Result<crate::tape::Tape, (Span, String)> {
        let ast = crate::expr::parse(text).map_err(|e| (span, e))?;
        crate::tape::Tape::compile(&ast.body, &vars).map_err(|e| (span, e))
    };
    let x = tape(&f.x, f.xspan)?;
    let y = tape(&f.y, f.yspan)?;
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
        x,
        y,
        domain,
    })
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
    let spec = r.kind.spec();
    let mut args: Vec<CArg> = Vec::with_capacity(spec.len());
    let mut left_out = vec![false; spec.len()];
    for (i, (name, kind)) in spec.iter().enumerate() {
        let given = r.args.get(i).and_then(|a| a.as_ref());
        let Some(a) = given else {
            left_out[i] = true;
            args.push(r.kind.default_arg(i));
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
    // the inferred slots the source left out — read off the geometry, the one place that rule
    // lives, shared with the document reader and the bindings' constraint records
    io::seed_omitted(sk, r.kind, &mut args, |i| left_out[i]);
    Some(sk.add_quiet(Constraint::new(r.kind, args)))
}

fn arg_span(a: &Arg) -> Option<Span> {
    match a {
        Arg::Ref(r) => Some(r.span),
        Arg::Dim { span, .. } => Some(*span),
        _ => None,
    }
}

fn to_arg(sk: &Sketch, res: &Resolver, kind: SpecKind, a: &Arg) -> Result<CArg, String> {
    Ok(match (kind, a) {
        (k, Arg::Ref(r)) if k.is_entity() => {
            let e = res
                .lookup(r)
                .ok_or_else(|| format!("no such entity: `{}`", r.root.text))?;
            let e = follow(sk, e, &r.path)?;
            if !crate::constraints::kind_matches(k, e.kind) {
                return Err(format!(
                    "`{}` is a {}, and a {} is wanted here",
                    r.root.text,
                    e.kind.as_str(),
                    k.as_str()
                ));
            }
            CArg::Ent(e)
        }
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
        (SpecKind::Int, Arg::Int(v)) => CArg::Int(*v),
        (SpecKind::Int, Arg::Num(v)) => CArg::Int(*v as i64),
        (SpecKind::Bool, Arg::Bool(b)) => CArg::Bool(*b),
        (SpecKind::Str, Arg::Word(w)) => CArg::Str(w.clone()),
        (SpecKind::Float, Arg::Num(v)) => CArg::Num(*v),
        (SpecKind::Float, Arg::Int(v)) => CArg::Num(*v as f64),
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

fn lift_decl(sk: &Sketch, e: EntRef) -> Decl {
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
        seed,
        def: (e.kind == EntKind::Curve)
            .then(|| Name::new(sk.curve_defs[sk.curves[e.i()].def as usize].name.clone())),
        values: Vec::new(),
        domain: None,
        knots,
        construction: construction_of(sk, e),
    }
}

fn construction_of(sk: &Sketch, e: EntRef) -> bool {
    match e.kind {
        EntKind::Line => sk.lines[e.i()].construction,
        EntKind::Curve => sk.curves[e.i()].construction,
        EntKind::Circle => sk.circles[e.i()].construction,
        EntKind::Arc => sk.arcs[e.i()].construction,
        EntKind::Spline => sk.splines[e.i()].construction,
        EntKind::Ellipse => sk.ellipses[e.i()].construction,
        EntKind::Point => false,
    }
}

fn lift_relation(sk: &Sketch, c: &Constraint) -> Relation {
    let spec = c.kind.spec();
    let mut args: Vec<Option<Arg>> = Vec::with_capacity(spec.len());
    for (i, (_, kind)) in spec.iter().enumerate() {
        args.push(lift_arg(sk, *kind, &c.args[i]));
    }
    Relation { kind: c.kind, args, place: sk.placements.get(&c.id).copied() }
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
