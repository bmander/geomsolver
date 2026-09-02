//! Instance expansion: a program with components and repetition becomes a flat list of statements.
//!
//! This is spec §14.1's first phase, and it is where the language's structure *goes*.  Everything
//! after it — building the geometry, adding the constraints, evaluating the expressions — works on
//! a list with no components and no blocks in it, exactly as it did before either existed.
//!
//! **Connection is aliasing, and aliasing is free.**  Binding a port or passing an entity to an
//! instance does not add a constraint; it makes two names denote one entity, which costs no
//! residual and cannot be violated (spec P1).  That is not an optimisation — it is why a
//! component boundary is free, and it needs nothing from the model at all: a name resolves to an
//! entity, and several names may resolve to the same one.
//!
//! Names go out of the top absolute — `t#3.lead` — and are never printed.  A program is printed
//! by lifting the *sketch* it elaborated to (`program::to_program`), which mints `p0`, `l1` and
//! the rest, so an internal name has no spelling to keep valid.

use crate::expr::{self, Aff};
use crate::model::EntKind;
use crate::program::Code;
use crate::units::Units;
use crate::syntax::{
    build_rank, under_root, BlockKind, Component, CurveTarget, Decl, DeclName, Kid, Name,
    OpenJoint, OpenNamed, OpenSide, Program, Ref, Seg, Span, Stmt, StmtKind, Ty,
};
use std::collections::{BTreeMap, BTreeSet};

/// How deep components and blocks may nest.  A document is untrusted input and
/// `wasm32-unknown-unknown` aborts rather than unwinding, so recursion is bounded here.
pub const MAX_DEPTH: usize = 32;

/// How many statements an expansion may produce.  `repeat 1000000` is a program somebody can
/// write; running out of memory is not the answer it should get.
pub const MAX_FLAT: usize = 200_000;

/// One elaborated-into-place statement, and what its names mean.
pub struct Flat {
    pub stmt: Stmt,
    /// The instance path that reached it: which copy of which block, outermost first.
    pub path: Vec<u32>,
}

/// What a `next` or a `prev` means where a statement stands.
#[derive(Clone)]
struct Cyc {
    prefix: String,
    k: usize,
    n: usize,
}

/// The `ring` a statement stands inside, if any: the block's prefix (every copy's name starts
/// with it), the axis as written, and where the block is.  What E021 is judged against.
#[derive(Clone)]
struct Ring {
    prefix: String,
    about: Ref,
}

/// What the names in one statement are resolved against: the prefixes it is nested in, innermost
/// first.  An instance's entity arguments are not here: a formal is a *port alias* under the
/// instance's own prefix (`bind`), found by `lookup` through the prefixes like any other name.
#[derive(Clone, Default)]
struct Scope {
    prefixes: Vec<String>,
    cyc: Option<Cyc>,
    ring: Option<Ring>,
    /// Whether a `cycle` or a `repeat` stands anywhere above: the prefix in force then carries a
    /// block's id (`#3.0.`) rather than an instance's name, and a declaration under it is one
    /// *copy* — shown and selected by, never written into a statement.  This walk is the only
    /// place that is known (`syntax::Named`, issue #39).
    copies: bool,
    /// The numbers in force where the statement was written — the enclosing counts, params and
    /// block binders.  An index (`p[i + 1]`) is an expression over exactly these, and references
    /// are resolved in a later pass where the walk's own environment is gone, so it travels here.
    vals: BTreeMap<String, Aff>,
    /// The view an enclosing instance is drawn `in` (§6.7): every point-bearing declaration
    /// emitted under it joins the plane.  The ref as *written* at the instance, with the
    /// prefixes of the scope it was written in — `rewrite` resolves it there and not in the
    /// component's own chain, where a body declaration called `top` would take it (#45.4).
    in_plane: Option<InPlane>,
}

impl Scope {
    /// The innermost prefix — what a name declared here is put under.
    fn prefix(&self) -> &str {
        self.prefixes.first().map(String::as_str).unwrap_or("")
    }
}

/// An instance's `in PLANE`, and where it was written — see `Scope::in_plane`.
#[derive(Clone)]
struct InPlane {
    plane: Ref,
    prefixes: Vec<String>,
}

pub struct Expansion {
    pub flat: Vec<Flat>,
    pub errors: Vec<(Span, String)>,
    /// Diagnostics that carry their own code: a `ring`'s (§12.3–12.6), which are not the
    /// "no such name" / "not a shape" pair the plain errors sort into.
    pub coded: Vec<(Code, Span, String)>,
    /// Every instance the walk bound, drawn or not — what a curve is a curve *of* (§6.5): the
    /// elaborator finds the instance a curve's point belongs to here, with the entities it was
    /// given resolved to absolute names and the numbers it was given worked out.
    pub instances: Vec<InstanceInfo>,
    /// `port x = y`, resolved: absolute name to absolute name.  What a curve's point is when
    /// the component exported it under another name.
    pub aliases: BTreeMap<String, String>,
}

/// One instance, as bound: which component, under what prefix, given what.
#[derive(Clone, Debug)]
pub struct InstanceInfo {
    /// The absolute prefix every name of its expansion starts with — `leg.`, `g.t.r.`, or
    /// `#c12.` for an instance written in place inside a curve.
    pub prefix: String,
    pub component: String,
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
    prog: &'a Program,
    /// What the document's numbers are in — carried through the walk because every expression
    /// worked out here is worked out in them.
    units: Units,
    out: Vec<(Stmt, Vec<u32>, Scope)>,
    /// Every absolute name a declaration will make.  Collected as the walk goes and used to
    /// resolve references afterwards, so forward reference works — which spec P2 requires, since
    /// a body is a set and a set has no "before".
    names: BTreeSet<String>,
    /// `port x = y`: one name for what another names.  Resolved transitively, after the walk.
    aliases: Vec<(String, Ref, Scope)>,
    instances: Vec<InstanceInfo>,
    /// `Some` while a component is expanded over its formals as variables — see `Sym`.
    sym: Option<Sym>,
    errors: Vec<(Span, String)>,
    coded: Vec<(Code, Span, String)>,
}

/// Expand a program's root component into a flat list of declarations, constraints, gauges and
/// orientations, with every name made absolute.
pub fn expand(prog: &Program, units: Units) -> Expansion {
    let mut w = Walk::new(prog, units, None);
    let root = prog.root();
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
    let scope = Scope { prefixes: vec![String::new()], ..Scope::default() };
    let mut vals: BTreeMap<String, Aff> = BTreeMap::new();
    for f in &comp.formals {
        match f.ty {
            Ty::Ent(_) => {
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

/// The drawing's unknown a formal stands for when nothing binds it — the rule that a name
/// nothing defines is a free variable, applied to a formal under its declared dimension.
fn free(name: String, ty: Ty) -> Aff {
    Aff { free: Some(name), m: 1.0, c: 0.0, dim: ty.dim() }
}

impl<'a> Walk<'a> {
    fn new(prog: &'a Program, units: Units, sym: Option<Sym>) -> Walk<'a> {
        Walk {
            prog,
            units,
            out: Vec::new(),
            names: BTreeSet::new(),
            aliases: Vec::new(),
            instances: Vec::new(),
            sym,
            errors: Vec::new(),
            coded: Vec::new(),
        }
    }

    fn err(&mut self, span: Span, msg: impl Into<String>) {
        if self.errors.len() < 200 {
            self.errors.push((span, msg.into()));
        }
    }

    fn finish(mut self) -> Expansion {
        let (flat, aliases) = self.resolve();
        Expansion {
            flat,
            errors: self.errors,
            coded: self.coded,
            instances: self.instances,
            aliases,
        }
    }

    /// A text with what `substitute` writes in, and — in the symbolic mode — every name that
    /// came to text (`Sym::texts`, looked up through the scope's prefixes like any other name)
    /// written in as well.  A kept text was itself substituted when it was kept, so one pass
    /// is the fixed point.  The identity outside the symbolic mode.
    fn subst_sym(&self, text: &str, vals: &BTreeMap<String, Aff>, scope: &Scope) -> String {
        let Some(sym) = &self.sym else { return text.to_string() };
        substitute_with(text, |w| {
            of_vals(vals)(w).or_else(|| {
                scope.prefixes.iter().find_map(|p| sym.texts.get(&format!("{p}{w}")).cloned())
            })
        })
    }

    /// A dimension's text, settled: the component's numbers written in, and the names that came
    /// to text as well.
    fn settle_text(
        &self,
        text: &str,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
    ) -> Result<String, String> {
        settle(&self.subst_sym(text, vals, scope), vals, self.units)
    }

    /// Keep a name as text — a value nothing here can work out, written in where it is read.
    /// The symbolic mode's one policy; `false` outside it, where the caller reports instead.
    fn keep_text(
        &mut self,
        abs: String,
        text: &str,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
    ) -> bool {
        let t = format!("({})", self.subst_sym(text, vals, scope).trim());
        match self.sym.as_mut() {
            Some(sym) => {
                sym.texts.insert(abs, t);
                true
            }
            None => false,
        }
    }

    /// One argument, read over the parameters in scope: a dimension's text settled to text, a
    /// seed or a pin settled to its number.
    ///
    /// **The one walk**, because a statement carries its arguments twice — as the operator was
    /// written and in spec order — and both halves are `syntax::Arg`.  Written twice, a new kind
    /// of argument gets settled in one of them and silently keeps the component's own names in
    /// the other.
    fn settle_arg(&mut self, a: &mut crate::syntax::Arg, vals: &BTreeMap<String, Aff>, scope: &Scope) {
        match a {
            crate::syntax::Arg::Dim { text, span } => match self.settle_text(text, vals, scope) {
                Ok(t) => *text = t,
                Err(e) => self.err(*span, format!("`{text}`: {e}")),
            },
            crate::syntax::Arg::SeedExpr { text, pinned, span } => {
                match value_of(text, vals, self.units) {
                    Ok(v) => *a = crate::syntax::Arg::Seed { value: v, pinned: *pinned },
                    Err(e) => self.err(*span, format!("`{text}`: {e}")),
                }
            }
            _ => {}
        }
    }

    /// The view an enclosing instance is drawn `in`, put on one declaration its expansion
    /// makes (§6.7).  A datum or a curve is left alone — it has no points of its own to put
    /// there — and a declaration that already says which plane (a clause of its own, on a
    /// plane the component declares) may not be told twice.
    fn stamp_scope_plane(&mut self, d: &mut Decl, scope: &Scope) {
        let Some(p) = &scope.in_plane else { return };
        if !d.kind.bears_points() {
            return;   // a datum's points are the datum's, and a curve is its expressions
        }
        if !d.membership.join(&p.plane, crate::syntax::Source::Instance) {
            self.err(d.membership.span(), d.membership.cause().to_string());
        }
    }

    /// Work out every `param` a body declares, before any statement of the body is walked.
    ///
    /// A body is a set (spec P2): `param h = w / 2` may stand above `param w = 60`, so the
    /// definitions are taken in *dependency* order and not in line order — a param is ready
    /// when none of the names it reads is another param of this body still waiting, and the
    /// ready ones are worked out until none is left.  What remains then reads itself, through
    /// however many others, which is the cyclic definitional dependency spec §11 names E041.
    /// A second `param w` in one body is the E001 a second `point w` is, and the first stands
    /// (#43.13); a param whose definition fails is reported once, where it is written, and the
    /// params that read it are left unsaid rather than each repeating the cause (#45.1).
    fn params(&mut self, body: &[Stmt], vals: &mut BTreeMap<String, Aff>, scope: &Scope) {
        let prefix = scope.prefix().to_string();
        let mut pending: Vec<&crate::syntax::ParamDecl> = Vec::new();
        let mut here: BTreeSet<String> = BTreeSet::new();
        for st in body {
            if let StmtKind::Param(pd) = &st.kind {
                if !here.insert(pd.name.text.clone()) {
                    self.err(pd.name.span, format!("`{}` is declared twice", pd.name.text));
                    continue;
                }
                pending.push(pd);
            }
        }
        // the names each definition reads; a text that does not parse reads nothing, and is
        // worked out at once so the parse error is the one reported
        let reads: Vec<BTreeSet<String>> = pending
            .iter()
            .map(|pd| {
                expr::parse_in(pd.text.trim(), self.units)
                    .map(|p| p.body.deps())
                    .unwrap_or_default()
            })
            .collect();
        let mut waiting: Vec<usize> = (0..pending.len()).collect();
        let mut failed: BTreeSet<String> = BTreeSet::new();
        loop {
            let names: BTreeSet<&str> =
                waiting.iter().map(|&i| pending[i].name.text.as_str()).collect();
            let (ready, rest): (Vec<usize>, Vec<usize>) = waiting
                .iter()
                .partition(|&&i| !reads[i].iter().any(|n| names.contains(n.as_str())));
            if ready.is_empty() {
                for i in rest {
                    let pd = pending[i];
                    let through = reads[i]
                        .iter()
                        .find(|n| names.contains(n.as_str()) && **n != pd.name.text)
                        .map(|n| format!(", through `{n}`"))
                        .unwrap_or_default();
                    self.coded.push((
                        Code::E041,
                        pd.span,
                        format!("`{}` is defined in terms of itself{through}", pd.name.text),
                    ));
                }
                return;
            }
            for i in ready {
                let pd = pending[i];
                if reads[i].iter().any(|n| failed.contains(n)) {
                    failed.insert(pd.name.text.clone());
                    continue;   // the cause is already reported, at the param it reads
                }
                match value_aff(&pd.text, vals, self.units) {
                    Ok(a) => {
                        vals.insert(pd.name.text.clone(), a);
                    }
                    // a text a curve's variables leave no value to — kept, in the symbolic
                    // mode; a mistake, on the sheet
                    Err(e) => {
                        let abs = format!("{prefix}{}", pd.name.text);
                        if !self.keep_text(abs, &pd.text, vals, scope) {
                            self.err(pd.span, format!("`{}`: {e}", pd.name.text));
                            failed.insert(pd.name.text.clone());
                        }
                    }
                }
            }
            waiting = rest;
        }
    }

    /// Walk one body, in source order.
    fn body(
        &mut self,
        body: &[Stmt],
        scope: &Scope,
        vals: &mut BTreeMap<String, Aff>,
        path: &[u32],
        depth: usize,
    ) {
        if depth > MAX_DEPTH {
            if let Some(st) = body.first() {
                self.err(st.span, format!("nested more than {MAX_DEPTH} deep"));
            }
            return;
        }
        let prefix = scope.prefix().to_string();
        // every `param` of the body first, whatever line it stands on — a body is a set (P2)
        self.params(body, vals, scope);
        // and with them the numbers in force are complete for every statement of the body:
        // the enclosing ones the caller passed and the body's own — so that is the table each
        // statement is emitted with, which is what an index (`p[n - 1]`) is read against.  The
        // root's scope arrived with an empty one, and a top-level index could read a literal
        // and not a `param` (#45.2).
        let scope = &Scope { vals: vals.clone(), ..scope.clone() };
        for st in body {
            if self.out.len() >= MAX_FLAT {
                self.err(st.span, format!("more than {MAX_FLAT} statements once expanded"));
                return;
            }
            match &st.kind {
                StmtKind::Decl(d) => {
                    let abs = format!("{prefix}{}", d.name.key().text);
                    self.names.insert(abs.clone());
                    let mut d2 = d.clone();
                    d2.name = d.name.prefixed(abs, scope.copies);
                    if let Some(c) = d2.curve.as_mut() {
                        // the interval is written over the parameters in scope, like a number
                        for t in [&mut c.domain.0, &mut c.domain.1] {
                            match value_of(t, vals, self.units) {
                                Ok(v) => *t = crate::syntax::num(v),
                                Err(e) => self.err(st.span, format!("`{t}`: {e}")),
                            }
                        }
                        // an instance written in place is bound like any other and never
                        // drawn: the curve is the only thing made of it (§6.5)
                        if let CurveTarget::Anon(inst, point) = &c.target {
                            if let Some((_, _, key)) = self.bind_instance(inst, scope, vals, false) {
                                c.of = Some(crate::syntax::CurveOf {
                                    instance: key,
                                    point: written(point),
                                });
                            }
                        }
                    }
                    self.settle_seeds(&mut d2, vals, scope, st.span);
                    // a plane's fold and basis are written over the parameters in scope like
                    // any other number, through the one walk that settles an argument
                    for a in d2.attitude.args_mut() {
                        self.settle_arg(a, vals, scope);
                    }
                    self.stamp_scope_plane(&mut d2, scope);
                    self.emit(StmtKind::Decl(d2), st, scope, path);
                }
                // `port lead: Point` is a fresh declaration that the boundary also names.  There
                // is nothing else to it: a port carries no joint, no direction and no constraint
                // — and its seed, where it wrote one, is a declaration's seed.
                StmtKind::Port(p) => {
                    if let Some(kind) = p.declare {
                        let abs = format!("{prefix}{}", p.name.text);
                        self.names.insert(abs.clone());
                        let mut d = Decl {
                            kind,
                            // the port's own written name, under the prefix — the same rule
                            // every ordinary declaration goes through above
                            name: DeclName::Written(p.name.clone()).prefixed(abs, scope.copies),
                            children: vec![Vec::new(); count_children(kind)],
                            seed: p.seed.clone(),
                            seed_text: p.seed_text.clone(),
                            seed_spans: p.seed_spans.clone(),
                            hint_span: p.hint_span,
                            knots: None,
                            curve: None,
                            class: Default::default(),
                            class_span: Span::default(),
                            seed_at: None,
                            attitude: Default::default(),
                            membership: Default::default(),
                            list_span: Span::default(),
                        };
                        self.settle_seeds(&mut d, vals, scope, st.span);
                        self.stamp_scope_plane(&mut d, scope);
                        self.emit(StmtKind::Decl(d), st, scope, path);
                    } else if let Some(r) = &p.alias {
                        let abs = format!("{prefix}{}", p.name.text);
                        self.aliases.push((abs, r.clone(), scope.clone()));
                    } else if let Some(xy) = &p.computed {
                        // a computed point is made of expressions over the formals, which is
                        // a thing a curve can be and a drawing cannot: nothing on the sheet
                        // holds a point to a formula (§6.5)
                        if self.sym.is_none() {
                            self.err(
                                p.name.span,
                                format!(
                                    "`{}` is a computed point, so its component is drawn only \
                                     as a curve: `curve e = Component(…).{} over u in (a, b)`",
                                    p.name.text, p.name.text
                                ),
                            );
                            continue;
                        }
                        let mut p2 = p.clone();
                        p2.name = Name { text: format!("{prefix}{}", p.name.text), span: p.name.span };
                        let [(x, xs), (y, ys)] = xy;
                        p2.computed = Some([
                            (self.subst_sym(x, vals, scope), *xs),
                            (self.subst_sym(y, vals, scope), *ys),
                        ]);
                        self.emit(StmtKind::Port(p2), st, scope, path);
                    }
                }
                StmtKind::Param(_) => {}   // worked out above, before the walk
                StmtKind::Instance(inst) => {
                    let Some((comp, mut sub_vals, key)) = self.bind_instance(inst, scope, vals, true)
                    else {
                        continue;
                    };
                    // the instance's own `in`, or the one already in force around it — both at
                    // once is a plane given twice, which one statement may not do (§6.7)
                    let in_plane = match (inst.membership.plane(), &scope.in_plane) {
                        (Some(p), Some(_)) => {
                            self.err(p.span, inst.membership.cause().to_string());
                            scope.in_plane.clone()
                        }
                        // written here, in this scope: it resolves against these prefixes
                        (Some(p), None) => {
                            Some(InPlane { plane: p.clone(), prefixes: scope.prefixes.clone() })
                        }
                        (None, q) => q.clone(),
                    };
                    let mut sc = Scope {
                        prefixes: std::iter::once(key).chain(scope.prefixes.iter().cloned()).collect(),
                        cyc: None,
                        ring: scope.ring.clone(),
                        copies: scope.copies,
                        vals: sub_vals.clone(),
                        in_plane,
                    };
                    sc.cyc = scope.cyc.clone();
                    self.body(&comp.body, &sc, &mut sub_vals, path, depth + 1);
                }
                StmtKind::Block(b) => {
                    let n = match value_of(&b.count, vals, self.units) {
                        Ok(v) if v.is_finite() && v >= 0.0 => v.round() as usize,
                        Ok(v) => {
                            self.err(b.span, format!("`{}` is {v}, which is not a count", b.count));
                            continue;
                        }
                        Err(e) => {
                            self.err(b.span, format!("`{}`: {e}", b.count));
                            continue;
                        }
                    };
                    if n == 0 {
                        continue;
                    }
                    if n > MAX_FLAT {
                        self.err(b.span, format!("{n} copies is more than {MAX_FLAT}"));
                        continue;
                    }
                    let block_prefix = format!("{prefix}#{}.", st.id.0);
                    // A `ring` is **unrolled**: its copies are made as a `cycle`'s are, congruent
                    // by the numbers each was given and not held so.  Spec §12.3 permits that
                    // and [0.2] requires it be said wherever the DOF ledger is, which W112 is;
                    // §12.6 lets a nested ring be refused and forbids mis-solving it, and
                    // refusing is the honest one of the two (#43.8, #43.9).
                    let ring = match b.kind {
                        BlockKind::Ring if scope.ring.is_some() => {
                            self.coded.push((
                                Code::E022,
                                b.span,
                                "a `ring` inside a `ring` is not supported".to_string(),
                            ));
                            continue;
                        }
                        BlockKind::Ring => {
                            self.coded.push((
                                Code::W112,
                                b.span,
                                format!(
                                    "`ring {}` is unrolled: its copies are congruent by the \
                                     numbers each was given, not held so, and the DOF counts \
                                     every copy",
                                    b.count
                                ),
                            ));
                            b.about.clone().map(|about| Ring { prefix: block_prefix.clone(), about })
                        }
                        _ => scope.ring.clone(),
                    };
                    let mut ranges: Vec<(usize, usize)> = Vec::new();
                    for k in 0..n {
                        let mut sub = vals.clone();
                        if let Some(i) = &b.binder {
                            sub.insert(i.text.clone(), Aff::num(k as f64));
                        }
                        let sc = Scope {
                            prefixes: std::iter::once(format!("{block_prefix}{k}."))
                                .chain(scope.prefixes.iter().cloned())
                                .collect(),
                            // `next` and `prev` mean something only where the copies close
                            cyc: b.kind.wraps().then(|| Cyc {
                                prefix: block_prefix.clone(),
                                k,
                                n,
                            }),
                            ring: ring.clone(),
                            // the prefix just built is the block's id, so every declaration
                            // below is a copy, however deep and through however many instances
                            copies: true,
                            vals: sub.clone(),
                            in_plane: scope.in_plane.clone(),
                        };
                        let mut p2 = path.to_vec();
                        p2.push(k as u32);
                        let from = self.out.len();
                        self.body(&b.body, &sc, &mut sub, &p2, depth + 1);
                        // the trailing joint's relations, stated between this copy and the
                        // next: every copy for a cycle or a ring (the wrap seals the loop),
                        // all but the last for a repeat, whose final corner is simply not
                        // stated (issue #38).  The joint is the *block's* statement, so it
                        // gets the `cyc` a repeat's own body does not — a wrapping kind's
                        // scope already carries it.
                        if let Some(j) = &b.joint {
                            if b.kind.wraps() {
                                self.body(&j.stmts, &sc, &mut sub, &p2, depth + 1);
                            } else if k + 1 < n {
                                let sc2 = Scope {
                                    cyc: Some(Cyc { prefix: block_prefix.clone(), k, n }),
                                    ..sc.clone()
                                };
                                self.body(&j.stmts, &sc2, &mut sub, &p2, depth + 1);
                            }
                            ranges.push((from, self.out.len()));
                        }
                    }
                    if let Some(j) = &b.joint {
                        self.weld(j, b.kind, &block_prefix, n, &ranges);
                    }
                }
                // a constraint: its dimension is written in the component's own parameters, which
                // do not exist in the flat document, so they are worked out here
                StmtKind::Relation(rel) => {
                    let mut r2 = rel.clone();
                    // the number and the seeds a relation carries are written in the component's
                    // own parameters, which the flat document does not have.  A statement holds
                    // them twice over — as written (`poly`, whose operator has not been settled
                    // yet) and in spec order — but both are `syntax::Arg`, so both walks are the
                    // same one and a new kind of argument is settled in one place.
                    if let Some(w) = r2.poly.as_mut() {
                        for a in w.args.iter_mut() {
                            match a {
                                crate::syntax::OpArg::Slot { arg, .. } => {
                                    self.settle_arg(arg, vals, scope)
                                }
                                crate::syntax::OpArg::Named(_, arg) => {
                                    self.settle_arg(arg, vals, scope)
                                }
                                crate::syntax::OpArg::Dim(text, span) => {
                                    match self.settle_text(text, vals, scope) {
                                        Ok(t) => *text = t,
                                        Err(e) => self.err(*span, format!("`{text}`: {e}")),
                                    }
                                }
                                crate::syntax::OpArg::Ent(_) => {}
                            }
                        }
                    }
                    for a in r2.args.iter_mut().flatten() {
                        self.settle_arg(a, vals, scope);
                    }
                    self.emit(StmtKind::Relation(r2), st, scope, path);
                }
                // a gauge or an orientation: kept as written, resolved later
                other => self.emit(other.clone(), st, scope, path),
            }
        }
    }

    /// A seed written as an expression is worked out here, against the parameters in scope,
    /// and is a number from now on — for a declaration and for the declaring form of a port
    /// alike, since both carry the one `hint(…)` clause a seed is written in.
    fn settle_seeds(&mut self, d: &mut Decl, vals: &BTreeMap<String, Aff>, scope: &Scope, span: Span) {
        for i in 0..d.seed_text.len() {
            let Some(t) = d.seed_text[i].take() else { continue };
            let t = self.subst_sym(&t, vals, scope);
            match value_of(&t, vals, self.units) {
                Ok(v) => d.seed[i] = v,
                // over a variable of the curve, or a formal's coordinate the curve's table
                // names and nothing here can: kept as text for the compile to read
                Err(_) if self.sym.is_some() => d.seed_text[i] = Some(t),
                Err(e) => self.err(span, format!("`{t}`: {e}")),
            }
        }
        if let Some((b, _)) = d.seed_at.as_mut().and_then(|a| a.bearing.as_mut()) {
            *b = self.subst_sym(b, vals, scope);
        }
    }

    /// An expanded statement keeps the id of the statement it came from.
    ///
    /// It is **the same statement** — a `cycle` of thirty makes thirty things from one line of
    /// source, and the line is what a caret lands on, what a splice edits and what a span points
    /// at.  What tells the thirty apart is the `path`, which is already on every `Site`.
    ///
    /// Minting a fresh id per copy would make each look like a statement of its own, and every
    /// consumer that turns an id back into source — `Elaborated::retext`, `adopt`, `edit::remove`
    /// — would find nothing there.  The multiplicity that a fresh id used to hide is exactly what
    /// `commit_seeds` needs to see: an id reached more than once has no single pose to record.
    fn emit(&mut self, kind: StmtKind, st: &Stmt, scope: &Scope, path: &[u32]) {
        let stmt = Stmt { id: st.id, kind, span: st.span, chained: st.chained };
        self.out.push((stmt, path.to_vec(), scope.clone()));
    }

    /// The open joint's weld, stated per pair of copies: the shared boundary point's name is
    /// written into one side's child slot, exactly as an in-chain `thread` writes it (#38).
    ///
    /// Which side takes the name follows `thread`'s own doctrine.  A side that *declared* its
    /// boundary named the point, so the other side's slot is filled; where neither did, the
    /// **later-built** side's slot takes the earlier side's minted dotted name, because a slot
    /// resolves through what is already built (`follow_building` refuses a reach into an
    /// entity that is not) and build order is per kind, then flat statement order — across the
    /// copies, `(kind, copy, statement)`.  That is `builds_first` generalized: for an ordinary
    /// pair the next copy builds later, and at a cycle's wrap the first copy built long ago,
    /// so the seam is spelled `prev.…` looking back or `next.…` looking forward and no pair
    /// references a point that does not yet exist.
    fn weld(&mut self, j: &OpenJoint, kind: BlockKind, block_prefix: &str, n: usize, ranges: &[(usize, usize)]) {
        let pairs = if kind.wraps() { n } else { n.saturating_sub(1) };
        for i in 0..pairs {
            let k = (i + 1) % n;
            // the side whose slot takes the shared point's name: the side that declared its
            // boundary named it, so the other is filled — and where neither did, the mint
            // goes on the side built first, so the fill goes on the other
            let fill_first = match j.named {
                OpenNamed::First => false,
                OpenNamed::Last => true,
                OpenNamed::Neither => {
                    (build_rank(j.last.kind), i, j.last.stmt.0)
                        < (build_rank(j.first.kind), k, j.first.stmt.0)
                }
            };
            let (side, copy, from, root) = if fill_first {
                (&j.first, k, &j.last, "prev")
            } else {
                (&j.last, i, &j.first, "next")
            };
            let r = under_root(root, &from.boundary, j.span);
            self.fill(ranges[copy], side, r, block_prefix, copy, n);
        }
    }

    /// Write the shared point's name into the emitted clone of one side's declaration, and
    /// give the clone's scope the block's own `cyc`, so `next`/`prev` resolve there whatever
    /// the block's kind — the weld is the block's statement, not one the body wrote.
    fn fill(
        &mut self,
        range: (usize, usize),
        side: &OpenSide,
        r: Ref,
        block_prefix: &str,
        copy: usize,
        n: usize,
    ) {
        for idx in range.0..range.1 {
            let (stmt, _, sc) = &mut self.out[idx];
            if stmt.id != side.stmt {
                continue;
            }
            if let StmtKind::Decl(d) = &mut stmt.kind {
                if let Some(slot) = d.children.get_mut(side.slot) {
                    *slot = vec![Kid::Ref(r)];
                }
                // a wrapping kind's scope carries the block's `cyc` already; a repeat's does
                // not, and the fill just written needs `lookup`'s `next`/`prev` arm.  The
                // grant is per-statement, so in a repeat this one welded declaration's *own*
                // refs would resolve `next` too — the E020 rule bent for exactly the
                // statements the weld touches, since resolution has no narrower scope to
                // give one reference.
                if sc.cyc.is_none() {
                    sc.cyc = Some(Cyc { prefix: block_prefix.to_string(), k: copy, n });
                }
            }
            return;
        }
    }

    /// Find the component an instance names and bind its arguments, recording the instance
    /// (`InstanceInfo`) for the curves that may be written over it.  Returns the component, what
    /// it was given, and the instance's absolute prefix; `None`, reported, when there is no such
    /// component.
    fn bind_instance(
        &mut self,
        inst: &crate::syntax::Instance,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        drawn: bool,
    ) -> Option<(&'a Component, BTreeMap<String, Aff>, String)> {
        let Some(comp) = self.prog.component(&inst.component.text) else {
            self.err(
                inst.component.span,
                format!("no component named `{}`", inst.component.text),
            );
            return None;
        };
        let sub_vals = self.bind(comp, inst, scope, vals);
        let key = format!("{}{}.", scope.prefix(), inst.name.text);
        self.instances.push(InstanceInfo {
            prefix: key.clone(),
            component: inst.component.text.clone(),
            ents: comp
                .formals
                .iter()
                .filter(|f| matches!(f.ty, Ty::Ent(_)))
                .map(|f| (f.name.text.clone(), None))
                .collect(),
            values: sub_vals.clone(),
            drawn,
        });
        Some((comp, sub_vals, key))
    }

    /// Bind an instantiation's arguments to the component's formals.
    ///
    /// An entity argument *aliases*: the formal and the actual denote one entity, at no cost.  A
    /// value argument is worked out here and is a number from then on.
    ///
    /// The alias is recorded exactly as `port f = actual` written in the instance's body would
    /// be: under the instance's **absolute** prefix, `{prefix}{inst}.{formal}`, resolved in the
    /// caller's scope.  Absolute, because that is the one key every reader already reaches —
    /// `lookup` walks a statement's prefixes outward, so the formal is found from a `repeat` in
    /// the body (`f.#3.1.hub` misses, `f.hub` hits), from a nested instance's own arguments
    /// (`Inner(q)` resolves `q` in `Outer`'s scope, where `o.q` is an alias), and separately per
    /// copy of an instance inside a block (`#3.0.s.b` and `#3.1.s.b` are two keys).  Keyed by
    /// the instance's bare name, as it once was, all three of those collapsed: a block's prefix
    /// was read back as the instance name, an outer formal was not yet bound when the inner
    /// alias looked for it, and three copies of `s` wrote one key, so every copy was bound to
    /// the last actual and the drawing came out silently wrong (issue #43).
    fn bind(
        &mut self,
        comp: &Component,
        inst: &crate::syntax::Instance,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
    ) -> BTreeMap<String, Aff> {
        use crate::syntax::{InstVal, Ty};
        let prefix = scope.prefix().to_string();
        let mut sub: BTreeMap<String, Aff> = BTreeMap::new();
        let mut positional = 0usize;
        for a in &inst.args {
            let formal = match &a.label {
                Some(l) => comp.formals.iter().find(|f| f.name.text == l.text),
                None => {
                    let f = comp.formals.get(positional);
                    positional += 1;
                    f
                }
            };
            let Some(f) = formal else {
                self.err(a.span, format!("`{}` has no such parameter", inst.component.text));
                continue;
            };
            match (&f.ty, &a.value) {
                (Ty::Ent(_), InstVal::Ref(r)) => {
                    // recorded unresolved; the resolve pass turns it into an absolute name in the
                    // *caller's* scope, which is what makes it an alias rather than a copy
                    self.aliases.push((
                        format!("{prefix}{}.{}", inst.name.text, f.name.text),
                        r.clone(),
                        scope.clone(),
                    ));
                }
                (Ty::Ent(_), InstVal::Expr(t)) => {
                    self.err(a.span, format!("`{}` wants an entity, and `{t}` is a number", f.name.text))
                }
                // A formal *declares* what its argument is — `phi: Angle`, `m: Length` — so the
                // number it stands for carries that dimension through the component's body.
                // This is where `param x = w + phi` is caught: nothing else in a component says
                // what a number is, and the substitution `settle` performs erases it.
                (ty, InstVal::Expr(t)) => {
                    self.bind_value(&mut sub, f, *ty, t, vals, scope, &inst.name.text, a.span)
                }
                (ty, InstVal::Ref(r)) => {
                    let t = r.root.text.clone();
                    self.bind_value(&mut sub, f, *ty, &t, vals, scope, &inst.name.text, a.span)
                }
            }
        }
        // A numeric formal the instance leaves unbound is an **unknown of the drawing**: the
        // language already makes a name nothing defines a free variable, and this is that
        // rule applied to a formal — a leg drawn with its crank angle unbound has a crank that
        // turns.  Named under the instance's own prefix (`leg.theta`), so two instances that
        // leave the same formal unbound have two unknowns and not one shared one — and inside
        // a traced component the name is no column of the curve, which is how a nested
        // instance's unbound formal is reported rather than captured by an outer one's.
        for f in &comp.formals {
            if matches!(f.ty, Ty::Ent(_)) || sub.contains_key(&f.name.text) {
                continue;
            }
            let name = format!("{prefix}{}.{}", inst.name.text, f.name.text);
            sub.insert(f.name.text.clone(), free(name, f.ty));
        }
        sub
    }

    /// One value argument, worked out and bound under the formal's *declared* dimension.
    ///
    /// The formal declares, so it wins — but an argument that said what it was and disagreed is
    /// reported: `Tooth(a0: 30mm)` is a mistake, not a conversion.
    #[allow(clippy::too_many_arguments)]
    fn bind_value(
        &mut self,
        sub: &mut BTreeMap<String, Aff>,
        f: &crate::syntax::Formal,
        ty: Ty,
        text: &str,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
        inst: &str,
        span: Span,
    ) {
        let want = ty.dim();
        match value_aff(text, vals, self.units) {
            Ok(a) => match a.dim.require(want, &f.name.text) {
                Ok(()) => {
                    sub.insert(f.name.text.clone(), a.as_dim(want));
                }
                Err(e) => self.err(span, e),
            },
            // a text a curve's variables leave no value to — kept, in the symbolic mode,
            // under the name the formal has inside the instance; a mistake, on the sheet
            Err(e) => {
                let abs = format!("{}{inst}.{}", scope.prefix(), f.name.text);
                if !self.keep_text(abs, text, vals, scope) {
                    self.err(span, format!("`{}`: {e}", f.name.text));
                }
            }
        }
    }

    /// Turn every reference into the absolute name of what it denotes.
    fn resolve(&mut self) -> (Vec<Flat>, BTreeMap<String, String>) {
        // port aliases first, and transitively: `port a = b` where `b` is itself a port
        let mut alias: BTreeMap<String, String> = BTreeMap::new();
        for (abs, r, sc) in self.aliases.clone() {
            if let Some((target, _)) = lookup(&r, &sc, &self.names, &alias, self.units) {
                alias.insert(abs, target);
            }
        }
        for _ in 0..MAX_DEPTH {
            let mut moved = false;
            let keys: Vec<String> = alias.keys().cloned().collect();
            for k in keys {
                let v = alias[&k].clone();
                if let Some(next) = alias.get(&v).cloned() {
                    if next != v {
                        alias.insert(k, next);
                        moved = true;
                    }
                }
            }
            if !moved {
                break;
            }
        }
        // what each instance was given, now that the aliases its arguments made are absolute
        for info in self.instances.iter_mut() {
            for (formal, actual) in info.ents.iter_mut() {
                *actual = alias.get(&format!("{}{formal}", info.prefix)).cloned();
            }
        }
        let out = std::mem::take(&mut self.out);
        let mut done: Vec<(Stmt, Vec<u32>, Scope, Option<StmtKind>)> = Vec::with_capacity(out.len());
        for (mut st, path, sc) in out {
            // a ring statement is judged after the rewrite (E021 below), against what its
            // references resolved to — but named in the message as they were written
            let written = sc.ring.as_ref().map(|_| st.kind.clone());
            let mut bad: Vec<(Span, String)> = Vec::new();
            rewrite(&mut st.kind, &sc, &self.names, &alias, self.units, &mut bad);
            let clean = bad.is_empty();
            for (span, msg) in bad {
                self.err(span, msg);
            }
            // a curve of a drawn instance's point: the point's name is absolute now, and the
            // instance it belongs to is the innermost one whose component has the formal
            if let StmtKind::Decl(Decl { curve: Some(c), .. }) = &mut st.kind {
                if let (CurveTarget::Drawn(r), true) = (&c.target, clean) {
                    let abs = written_ref(r);
                    match self.owner_of(&abs, &c.swept.text) {
                        Some(of) => c.of = Some(of),
                        None => self.err(
                            r.span,
                            format!(
                                "`{abs}` is not a point of an instance whose component has a \
                                 numeric formal `{}`, and a curve is a point of a component \
                                 as one of its formals runs",
                                c.swept.text
                            ),
                        ),
                    }
                }
            }
            done.push((st, path, sc, written.filter(|_| clean)));
        }
        self.judge_rings(&done, &alias);
        let flat = done.into_iter().map(|(stmt, path, _, _)| Flat { stmt, path }).collect();
        (flat, alias)
    }

    /// The instance a curve of the point `abs` is a curve *of*: the innermost drawn instance
    /// whose prefix the name starts with **and whose component declares `swept` as a numeric
    /// formal** — so `o.i.t over u` sweeps `Outer`'s `u` when `Inner` has none, and `Inner`'s
    /// own when it does.
    fn owner_of(&self, abs: &str, swept: &str) -> Option<crate::syntax::CurveOf> {
        let mut owners: Vec<&InstanceInfo> =
            self.instances.iter().filter(|i| i.drawn && abs.starts_with(&i.prefix)).collect();
        owners.sort_by_key(|i| std::cmp::Reverse(i.prefix.len()));
        owners
            .into_iter()
            .find(|i| {
                self.prog.component(&i.component).is_some_and(|c| {
                    c.formals.iter().any(|f| f.name.text == swept && !matches!(f.ty, Ty::Ent(_)))
                })
            })
            .map(|i| crate::syntax::CurveOf {
                instance: i.prefix.clone(),
                point: abs[i.prefix.len()..].to_string(),
            })
    }

    /// E021 (spec §12.5): a statement inside a `ring` may reference, outside the ring, only
    /// what the ring's turn leaves where it is — the axis point, and a circle or an arc centred
    /// on it.  Anything else — a stray point every copy is dimensioned to — cannot be true of
    /// all N copies at once, and the spec calls the refusal one of the language's best
    /// diagnostics; without it the same document came back as eight `over` culprits.
    fn judge_rings(&mut self, done: &[(Stmt, Vec<u32>, Scope, Option<StmtKind>)], alias: &BTreeMap<String, String>) {
        if done.iter().all(|(_, _, sc, _)| sc.ring.is_none()) {
            return;
        }
        // what every declared entity is, and for a circle or an arc, the absolute name of its
        // centre — the first child, resolved by the rewrite above
        let mut decls: BTreeMap<&str, (EntKind, Option<&str>)> = BTreeMap::new();
        for (st, _, _, _) in done {
            if let StmtKind::Decl(d) = &st.kind {
                let centre = match d.kind {
                    EntKind::Circle | EntKind::Arc => d.children.first().and_then(|g| g.first()).and_then(|k| match k {
                        Kid::Ref(r) => Some(r.root.text.as_str()),
                        Kid::Hint(_) => None,
                    }),
                    _ => None,
                };
                decls.insert(d.name.key().text.as_str(), (d.kind, centre));
            }
        }
        for (st, _, sc, written) in done {
            let (Some(ring), Some(written)) = (&sc.ring, written) else { continue };
            // the axis, resolved where the block stands: outside every copy's own prefix
            let outer = Scope {
                prefixes: sc.prefixes.iter().filter(|p| !p.starts_with(&ring.prefix)).cloned().collect(),
                ..Scope::default()
            };
            let axis = lookup(&ring.about, &outer, &self.names, alias, self.units).map(|(a, _)| a);
            for (now, was) in refs_of(&st.kind).into_iter().zip(refs_of(written)) {
                let x = now.root.text.as_str();
                if x.starts_with(&ring.prefix) || Some(x) == axis.as_deref() {
                    continue;
                }
                // a plane is a label on the sheet, not a position: a membership or a fold
                // referencing one from inside a ring is true of every copy alike
                if matches!(decls.get(x), Some((EntKind::Plane, _))) {
                    continue;
                }
                let invariant = matches!(
                    decls.get(x),
                    Some((EntKind::Circle | EntKind::Arc, Some(c))) if Some(*c) == axis.as_deref()
                );
                if !invariant {
                    self.coded.push((
                        Code::E021,
                        was.span,
                        format!(
                            "`{}` is outside the `ring` and does not turn with it: from inside, \
                             only the axis point and a circle or arc centred on it may be \
                             referenced",
                            written_ref(was)
                        ),
                    ));
                }
            }
        }
    }
}

fn count_children(k: EntKind) -> usize {
    k.fields().iter().filter(|(_, f)| *f != crate::model::Field::Scalar).count()
}

/// A number worked out while elaborating.
///
/// The language's angle is the degree — `sin(90)` is 1, and every dimension a person reads is in
/// degrees — so a whole turn is `tau = 360`, not 2π.  Keeping `tau` at 2π and the texts in degrees
/// would give two units one name; units say which of the two each is (`expr::CONSTANTS`), so
/// `pi` is the dimensionless constant and `tau` is an **angle**, and `tau == 2 * pi * 1rad`
/// holds where it used to be a coincidence of the same digits.
fn value_of(text: &str, env: &BTreeMap<String, Aff>, units: Units) -> Result<f64, String> {
    let a = value_aff(text, env, units)?;
    a.number().ok_or_else(|| {
        format!(
            "`{}` is not a number here — a component's parameters are, and the document's \
             dimensions are not",
            a.free.unwrap_or_default()
        )
    })
}

/// The same, keeping what it *is* beside what it is worth — and what it is *in terms of*: a
/// text over a formal left unbound comes to an affine value in that unknown (`bind`), which a
/// `param`, an argument to a nested instance and a dimension all carry on.  Only a caller that
/// needs a number (`value_of`: a seed, a count, an index) refuses one.
///
/// A `param`'s dimension has to survive into the names that read it, or `param R = m * N / 2`
/// would forget that `m` was declared a `Length` and `param Rt = R + m` would read as a plain
/// number added to a length — the very thing the check exists to catch, missed because the check
/// threw the answer away.
pub(crate) fn value_aff(
    text: &str,
    env: &BTreeMap<String, Aff>,
    units: Units,
) -> Result<Aff, String> {
    let t = text.trim();
    if t.is_empty() {
        return Err("nothing to work out".to_string());
    }
    let p = expr::parse_in(t, units)?;
    let a = expr::eval(&p.body, env)?;
    match (a.number(), &a.free) {
        (Some(v), _) if !v.is_finite() => Err(format!("comes to {v}")),
        // an unknown the scope *bound* — a formal left unbound, a param over one — carries on;
        // a name nothing binds is the document's, and a component's numbers cannot read it
        (None, Some(n)) if !env.values().any(|b| b.free.as_deref() == Some(n)) => Err(format!(
            "`{n}` is not a number here — a component's parameters are, and the document's \
             dimensions are not"
        )),
        _ => Ok(a),
    }
}

/// A dimension's text with the enclosing component's parameters worked out.
///
/// The text after `==` is `expr.rs`'s language and is evaluated against the *document's* named
/// dimensions — where a component's own parameters are not, and a name nothing defines is a free
/// variable there.  So `sin(half / 2)` inside a `Tooth` would quietly become an unknown for the
/// solver to answer rather than the number the component was given.  Substituting first is what
/// keeps the two namespaces apart.
///
/// A text that comes out a plain number is replaced by it: the formula was about parameters the
/// flat document no longer has, so printing it back would name things that are not there.  One
/// that still reads a document name — `w / 2` — keeps its form, and the reader keeps their
/// formula.
fn settle(text: &str, vals: &BTreeMap<String, Aff>, units: Units) -> Result<String, String> {
    let sub = substitute(text, vals);
    // nothing of the component's in it: leave it exactly as written, so `h = w / 2` and `3 1/8`
    // reach the document with the form somebody typed
    if sub == text {
        return Ok(text.to_string());
    }
    let p = expr::parse_in(&sub, units)?;
    let env: BTreeMap<String, Aff> = BTreeMap::new();
    if let Ok(a) = expr::eval(&p.body, &env) {
        if let Some(v) = a.number() {
            if v.is_finite() {
                // a definition keeps its name: it is the document's, not the component's
                return Ok(match &p.name {
                    Some(n) => format!("{n} = {}", crate::syntax::num(v)),
                    None => crate::syntax::num(v),
                });
            }
        }
    }
    Ok(sub)
}

/// Every identifier the environment knows, replaced by the number it stands for.
///
/// At identifier boundaries, so `flank` in `cos(flank)` is replaced and the `flank` inside
/// `flank_out` is not; and parenthesised, so a negative value does not change what binds to what.
fn substitute(text: &str, vals: &BTreeMap<String, Aff>) -> String {
    substitute_with(text, of_vals(vals))
}

/// What a word stands for in `vals`, as text: a number, parenthesised; a value affine in the
/// drawing's unknown (a formal left unbound, or a `param` over one) as `(m * name + c)`, or
/// the bare name when that is all it is.  `tau` and `turn` are `expr::CONSTANTS` now, and an
/// angle rather than a number that happens to be 360, so they are left for the evaluator.
fn of_vals(vals: &BTreeMap<String, Aff>) -> impl Fn(&str) -> Option<String> + '_ {
    move |w| {
        let a = vals.get(w)?;
        if let Some(v) = a.number() {
            return Some(format!("({})", crate::syntax::num(v)));
        }
        let n = a.free.as_ref()?;
        Some(if a.m == 1.0 && a.c == 0.0 {
            n.clone()
        } else {
            format!("({} * {n} + {})", crate::syntax::num(a.m), crate::syntax::num(a.c))
        })
    }
}

/// `substitute`, over whatever `of` says a word stands for.
fn substitute_with(text: &str, of: impl Fn(&str) -> Option<String>) -> String {
    let mut out = String::with_capacity(text.len());
    let b: Vec<char> = text.chars().collect();
    let mut i = 0usize;
    while i < b.len() {
        if b[i].is_alphabetic() || b[i] == '_' {
            let from = i;
            while i < b.len() && (b[i].is_alphanumeric() || b[i] == '_') {
                i += 1;
            }
            let word: String = b[from..i].iter().collect();
            match of(&word) {
                Some(t) => out.push_str(&t),
                None => out.push_str(&word),
            }
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    out
}



/// The absolute name `name` has inside copy `k` of whichever block declares it, under
/// `container` — the dotted path written before the indexed name (`l.` for `l.p[1]`, nothing
/// for `p[1]`), which is an instance the block was walked inside.
///
/// A block's copies are named `<enclosing>#<statement-id>.<k>.`, so this is a search for that
/// shape among the names the walk has already collected: for each scope the reference can see,
/// every block *within* the container there that declares `name`, at the copy asked for.  Two blocks in one scope
/// declaring the same name make the index ambiguous, and an ambiguous name resolves to nothing
/// rather than to whichever came first.
fn copy_of(
    container: &str,
    name: &str,
    k: usize,
    sc: &Scope,
    names: &BTreeSet<String>,
) -> Option<String> {
    let mut found: Option<String> = None;
    for p in &sc.prefixes {
        let from = format!("{p}{container}#");
        for abs in names.range(from.clone()..).take_while(|n| n.starts_with(&from)) {
            // `#<id>.<j>.<tail>` — the copy's own name, with no further block between
            let tail = &abs[from.len()..];
            let Some((_id, rest)) = tail.split_once('.') else { continue };
            let Some((j, leaf)) = rest.split_once('.') else { continue };
            if leaf != name || j.parse::<usize>() != Ok(k) {
                continue;
            }
            if found.as_deref().is_some_and(|f| f != abs) {
                return None; // two blocks here declare it: say nothing rather than guess
            }
            found = Some(abs.clone());
        }
        if found.is_some() {
            return found;
        }
    }
    found
}

/// The absolute name a reference denotes, and whatever field path is left over.
///
/// Greedy on the dotted name: `t.lead` is one name if something declared it, and `c0.center` is
/// the entity `c0` and its field `center`.  Which it is cannot be told from the spelling, only
/// from what exists — so the longest match that names something wins.
fn lookup(
    r: &Ref,
    sc: &Scope,
    names: &BTreeSet<String>,
    alias: &BTreeMap<String, String>,
    units: Units,
) -> Option<(String, Vec<String>)> {
    // `p[k]` names *which copy* of a repeated statement, so it is resolved before anything else
    // and the rest of the path is read against the copy it picks.  The index may stand on the
    // root or on a dotted name — `l.p[1]` is copy 1 of the `p` a block inside the instance `l`
    // declares, which is how a component's repetition is reached from outside (#45.3) — and
    // what stands before it is the *container* the block was walked under, never a field: an
    // index selects a copy of the block a name was declared in, and a field of a copy is still
    // a field, so it comes after.  Only once — a copy of a copy is a thing no statement makes.
    if let Some(at) = r.path.iter().position(|s| matches!(s, Seg::Index(_))) {
        let Seg::Index(text) = &r.path[at] else { unreachable!() };
        if r.path[at + 1..].iter().any(|s| matches!(s, Seg::Index(_))) {
            return None;
        }
        let k = match value_of(text, &sc.vals, units) {
            Ok(v) if v.is_finite() && v >= 0.0 && v.round() == v => v as usize,
            _ => return None,
        };
        let mut segs: Vec<&str> = vec![r.root.text.as_str()];
        for s in &r.path[..at] {
            if let Seg::Field(f) = s {
                segs.push(f.text.as_str());
            }
        }
        let leaf = segs.pop()?;
        let container = segs.iter().map(|s| format!("{s}.")).collect::<String>();
        let abs = copy_of(&container, leaf, k, sc, names)?;
        let rest: Vec<String> = r.path[at + 1..]
            .iter()
            .map(|s| match s {
                Seg::Field(f) => f.text.clone(),
                Seg::Index(t) => t.clone(),
            })
            .collect();
        return Some((abs, rest));
    }

    let mut segs: Vec<String> = vec![r.root.text.clone()];
    for s in &r.path {
        match s {
            Seg::Field(f) => segs.push(f.text.clone()),
            Seg::Index(t) => segs.push(t.clone()),
        }
    }
    // `next` and `prev` name the sibling copy, so the rest of the path is read in *its* scope
    let mut prefixes = sc.prefixes.clone();
    if (segs[0] == "next" || segs[0] == "prev") && segs.len() > 1 {
        let c = sc.cyc.as_ref()?;
        let k = match segs[0].as_str() {
            "next" => (c.k + 1) % c.n,
            _ => (c.k + c.n - 1) % c.n,
        };
        segs.remove(0);
        prefixes = vec![format!("{}{}.", c.prefix, k)];
    }
    for take in (1..=segs.len()).rev() {
        let cand = segs[..take].join(".");
        let rest: Vec<String> = segs[take..].to_vec();
        for p in &prefixes {
            let abs = format!("{p}{cand}");
            if names.contains(&abs) {
                return Some((abs, rest));
            }
            if let Some(t) = alias.get(&abs) {
                return Some((t.clone(), rest));
            }
        }
        if names.contains(&cand) {
            return Some((cand, rest));
        }
        if let Some(t) = alias.get(&cand) {
            return Some((t.clone(), rest));
        }
    }
    None
}

/// Every reference a statement makes, in the order `rewrite` visits them — the two walks must
/// agree, since `judge_rings` zips a statement before the rewrite with itself after.
fn refs_of(k: &StmtKind) -> Vec<&Ref> {
    let mut out = Vec::new();
    match k {
        StmtKind::Decl(d) => {
            for g in &d.children {
                for kid in g {
                    if let Kid::Ref(r) = kid {
                        out.push(r);
                    }
                }
            }
            // then the plane it is in and the plane it is folded from — `rewrite` visits them
            // in this order — then a curve's drawn target and a geometric seed's place
            out.extend(d.membership.plane());
            out.extend(d.attitude.plane_ref());
            if let Some(crate::syntax::CurveSpec { target: CurveTarget::Drawn(r), .. }) = &d.curve {
                out.push(r);
            }
            if let Some(at) = &d.seed_at {
                out.push(&at.what);
            }
        }
        StmtKind::Relation(rel) => {
            for a in rel.args.iter().flatten() {
                if let crate::syntax::Arg::Ref(r) = a {
                    out.push(r);
                }
            }
            if let Some(w) = &rel.poly {
                out.extend(w.ops.iter());
                for a in &w.args {
                    if let crate::syntax::OpArg::Ent(r) = a {
                        out.push(r);
                    }
                }
            }
        }
        StmtKind::Gauge(g) => match g {
            crate::syntax::Gauge::Ground(r) | crate::syntax::Gauge::Fix(r) => out.push(r),
        },
        StmtKind::Orient(o) => out.extend(o.pts.iter()),
        _ => {}
    }
    out
}

/// A reference spelled back the way the source wrote it, for a message about it.
fn written_ref(r: &Ref) -> String {
    written(r)
}

/// A reference spelled back the way the source wrote it, for a message about it.
fn written(r: &Ref) -> String {
    let mut out = r.root.text.clone();
    for seg in &r.path {
        match seg {
            Seg::Field(f) => {
                out.push('.');
                out.push_str(&f.text);
            }
            Seg::Index(t) => out.push_str(&format!("[{t}]")),
        }
    }
    out
}

fn rewrite(
    k: &mut StmtKind,
    sc: &Scope,
    names: &BTreeSet<String>,
    alias: &BTreeMap<String, String>,
    units: Units,
    bad: &mut Vec<(Span, String)>,
) {
    let fix = |r: &mut Ref, bad: &mut Vec<(Span, String)>| match lookup(r, sc, names, alias, units) {
        Some((abs, rest)) => {
            r.root = Name { text: abs, span: r.root.span };
            r.path = rest.into_iter().map(|f| Seg::Field(Name::new(f))).collect();
        }
        // named as written, so an index that picked no copy says which one it was
        None => bad.push((r.span, format!("no such entity: `{}`", written(r)))),
    };
    match k {
        StmtKind::Decl(d) => {
            for g in &mut d.children {
                for kid in g.iter_mut() {
                    // a seeded slot names nothing, so there is nothing in it to rescope
                    if let crate::syntax::Kid::Ref(r) = kid {
                        fix(r, bad);
                    }
                }
            }
            // a plane an instance gave the statement was written at the instance, in the
            // caller's scope, and is resolved there — the component's own names are not in
            // the caller's sight, so they may not take it (#45.4)
            let from_instance = d.membership.source() == crate::syntax::Source::Instance;
            if let Some(r) = d.membership.plane_mut() {
                match (&sc.in_plane, from_instance) {
                    (Some(ip), true) => {
                        let outer = Scope { prefixes: ip.prefixes.clone(), ..sc.clone() };
                        match lookup(r, &outer, names, alias, units) {
                            Some((abs, rest)) => {
                                r.root = Name { text: abs, span: r.root.span };
                                r.path =
                                    rest.into_iter().map(|f| Seg::Field(Name::new(f))).collect();
                            }
                            None => bad.push((r.span, format!("no such entity: `{}`", written(r)))),
                        }
                    }
                    _ => fix(r, bad),
                }
            }
            if let Some(r) = d.attitude.plane_ref_mut() {
                fix(r, bad);
            }
            if let Some(crate::syntax::CurveSpec { target: CurveTarget::Drawn(r), .. }) =
                d.curve.as_mut()
            {
                fix(r, bad);
            }
            if let Some(at) = d.seed_at.as_mut() {
                fix(&mut at.what, bad);
            }
        }
        StmtKind::Relation(rel) => {
            for a in rel.args.iter_mut().flatten() {
                if let crate::syntax::Arg::Ref(r) = a {
                    fix(r, bad);
                }
            }
            // an operator's operands are references like any other, and are the only ones a
            // written statement has — `args` is the *settled* form, which does not exist yet
            if let Some(w) = rel.poly.as_mut() {
                for r in w.ops.iter_mut() {
                    fix(r, bad);
                }
                for a in w.args.iter_mut() {
                    if let crate::syntax::OpArg::Ent(r) = a {
                        fix(r, bad);
                    }
                }
            }
        }
        StmtKind::Gauge(g) => match g {
            crate::syntax::Gauge::Ground(r) | crate::syntax::Gauge::Fix(r) => fix(r, bad),
        },
        StmtKind::Orient(o) => {
            for r in o.pts.iter_mut() {
                fix(r, bad);
            }
        }
        _ => {}
    }
}
