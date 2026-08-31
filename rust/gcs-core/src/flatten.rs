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
    build_rank, under_root, BlockKind, Component, Decl, DeclName, Kid, Name, OpenJoint,
    OpenNamed, OpenSide, Program, Ref, Seg, Span, Stmt, StmtKind, Ty,
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
    /// emitted under it joins the plane.  The ref as *written* at the instance — the emitted
    /// statement's own `rewrite` resolves it, reaching the caller's names through the prefix
    /// chain this scope already carries.
    in_plane: Option<Ref>,
}

pub struct Expansion {
    pub flat: Vec<Flat>,
    pub errors: Vec<(Span, String)>,
    /// Diagnostics that carry their own code: a `ring`'s (§12.3–12.6), which are not the
    /// "no such name" / "not a shape" pair the plain errors sort into.
    pub coded: Vec<(Code, Span, String)>,
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
    errors: Vec<(Span, String)>,
    coded: Vec<(Code, Span, String)>,
}

/// Expand a program's root component into a flat list of declarations, constraints, gauges and
/// orientations, with every name made absolute.
pub fn expand(prog: &Program, units: Units) -> Expansion {
    let mut w = Walk {
        prog,
        units,
        out: Vec::new(),
        names: BTreeSet::new(),
        aliases: Vec::new(),
        errors: Vec::new(),
        coded: Vec::new(),
    };
    let root = prog.root();
    let scope = Scope { prefixes: vec![String::new()], ..Scope::default() };
    let mut vals: BTreeMap<String, Aff> = BTreeMap::new();
    w.body(&root.body, &scope, &mut vals, &[], 0);
    let flat = w.resolve();
    Expansion { flat, errors: w.errors, coded: w.coded }
}

impl<'a> Walk<'a> {
    fn err(&mut self, span: Span, msg: impl Into<String>) {
        if self.errors.len() < 200 {
            self.errors.push((span, msg.into()));
        }
    }

    /// One argument, read over the parameters in scope: a dimension's text settled to text, a
    /// seed or a pin settled to its number.
    ///
    /// **The one walk**, because a statement carries its arguments twice — as the operator was
    /// written and in spec order — and both halves are `syntax::Arg`.  Written twice, a new kind
    /// of argument gets settled in one of them and silently keeps the component's own names in
    /// the other.
    fn settle_arg(&mut self, a: &mut crate::syntax::Arg, vals: &BTreeMap<String, Aff>) {
        match a {
            crate::syntax::Arg::Dim { text, span } => match settle(text, vals, self.units) {
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
        if !d.membership.join(p, crate::syntax::Source::Instance) {
            self.err(d.membership.span(), d.membership.cause().to_string());
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
        let prefix = scope.prefixes.first().cloned().unwrap_or_default();
        // the params this body has declared: a second `param w` in one body is the E001 a
        // second `point w` is, and the first stands (#43.13)
        let mut params_here: BTreeSet<String> = BTreeSet::new();
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
                    // a curve's numbers and the interval it is drawn over are written over the
                    // parameters in scope too, and are worked out in the same pass
                    for (_, t) in d2.values.iter_mut() {
                        match value_of(t, vals, self.units) {
                            Ok(v) => *t = crate::syntax::num(v),
                            Err(e) => self.err(st.span, format!("`{t}`: {e}")),
                        }
                    }
                    if let Some((a, b)) = d2.domain.as_mut() {
                        for t in [a, b] {
                            match value_of(t, vals, self.units) {
                                Ok(v) => *t = crate::syntax::num(v),
                                Err(e) => self.err(st.span, format!("`{t}`: {e}")),
                            }
                        }
                    }
                    self.settle_seeds(&mut d2, vals, st.span);
                    // a plane's fold and basis are written over the parameters in scope like
                    // any other number, through the one walk that settles an argument
                    for a in d2.attitude.args_mut() {
                        self.settle_arg(a, vals);
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
                            def: None,
                            values: Vec::new(),
                            domain: None,
                            class: Default::default(),
                            class_span: Span::default(),
                            seed_at: None,
                            attitude: Default::default(),
                            membership: Default::default(),
                            list_span: Span::default(),
                        };
                        self.settle_seeds(&mut d, vals, st.span);
                        self.stamp_scope_plane(&mut d, scope);
                        self.emit(StmtKind::Decl(d), st, scope, path);
                    } else if let Some(r) = &p.alias {
                        let abs = format!("{prefix}{}", p.name.text);
                        self.aliases.push((abs, r.clone(), scope.clone()));
                    }
                }
                StmtKind::Param(pd) => {
                    if !params_here.insert(pd.name.text.clone()) {
                        self.err(pd.name.span, format!("`{}` is declared twice", pd.name.text));
                        continue;
                    }
                    match value_aff(&pd.text, vals, self.units) {
                        Ok(a) => {
                            vals.insert(pd.name.text.clone(), a);
                        }
                        Err(e) => self.err(pd.span, format!("`{}`: {e}", pd.name.text)),
                    };
                }
                StmtKind::Instance(inst) => {
                    let Some(comp) = self.prog.components.iter().find(|c| {
                        c.name.as_ref().map(|n| n.text.as_str()) == Some(inst.component.text.as_str())
                    }) else {
                        self.err(
                            inst.component.span,
                            format!("no component named `{}`", inst.component.text),
                        );
                        continue;
                    };
                    let comp = comp.clone();
                    let mut sub_vals = self.bind(&comp, inst, scope, vals);
                    // the instance's own `in`, or the one already in force around it — both at
                    // once is a plane given twice, which one statement may not do (§6.7)
                    let in_plane = match (inst.membership.plane(), &scope.in_plane) {
                        (Some(p), Some(_)) => {
                            self.err(p.span, inst.membership.cause().to_string());
                            scope.in_plane.clone()
                        }
                        (Some(p), None) => Some(p.clone()),
                        (None, q) => q.clone(),
                    };
                    let mut sc = Scope {
                        prefixes: std::iter::once(format!("{prefix}{}.", inst.name.text))
                            .chain(scope.prefixes.iter().cloned())
                            .collect(),
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
                                    self.settle_arg(arg, vals)
                                }
                                crate::syntax::OpArg::Named(_, arg) => self.settle_arg(arg, vals),
                                crate::syntax::OpArg::Dim(text, span) => {
                                    match settle(text, vals, self.units) {
                                        Ok(t) => *text = t,
                                        Err(e) => self.err(*span, format!("`{text}`: {e}")),
                                    }
                                }
                                crate::syntax::OpArg::Ent(_) => {}
                            }
                        }
                    }
                    for a in r2.args.iter_mut().flatten() {
                        self.settle_arg(a, vals);
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
    fn settle_seeds(&mut self, d: &mut Decl, vals: &BTreeMap<String, Aff>, span: Span) {
        for i in 0..d.seed_text.len() {
            let Some(t) = d.seed_text[i].clone() else { continue };
            match value_of(&t, vals, self.units) {
                Ok(v) => d.seed[i] = v,
                Err(e) => self.err(span, format!("`{t}`: {e}")),
            }
            d.seed_text[i] = None;
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
        let prefix = scope.prefixes.first().cloned().unwrap_or_default();
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
                (ty, InstVal::Expr(t)) => self.bind_value(&mut sub, f, *ty, t, vals, a.span),
                (ty, InstVal::Ref(r)) => {
                    self.bind_value(&mut sub, f, *ty, &r.root.text, vals, a.span)
                }
            }
        }
        sub
    }

    /// One value argument, worked out and bound under the formal's *declared* dimension.
    ///
    /// The formal declares, so it wins — but an argument that said what it was and disagreed is
    /// reported: `Tooth(a0: 30mm)` is a mistake, not a conversion.
    fn bind_value(
        &mut self,
        sub: &mut BTreeMap<String, Aff>,
        f: &crate::syntax::Formal,
        ty: Ty,
        text: &str,
        vals: &BTreeMap<String, Aff>,
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
            Err(e) => self.err(span, format!("`{}`: {e}", f.name.text)),
        }
    }

    /// Turn every reference into the absolute name of what it denotes.
    fn resolve(&mut self) -> Vec<Flat> {
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
            done.push((st, path, sc, written.filter(|_| clean)));
        }
        self.judge_rings(&done, &alias);
        done.into_iter().map(|(stmt, path, _, _)| Flat { stmt, path }).collect()
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
    Ok(value_aff(text, env, units)?.c)
}

/// The same, keeping what it *is* beside what it is worth.
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
    match a.number() {
        Some(v) if v.is_finite() => Ok(a),
        Some(v) => Err(format!("comes to {v}")),
        None => Err(format!(
            "`{}` is not a number here — a component's parameters are, and the document's \
             dimensions are not",
            a.free.unwrap_or_default()
        )),
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
            // `tau` and `turn` are `expr::CONSTANTS` now, and an angle rather than a number
            // that happens to be 360, so they are left for the evaluator to read
            let known = vals.get(&word).and_then(|a| a.number());
            match known {
                Some(v) => out.push_str(&format!("({})", crate::syntax::num(v))),
                None => out.push_str(&word),
            }
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    out
}

/// The absolute name `name` has inside copy `k` of whichever block declares it.
///
/// A block's copies are named `<enclosing>#<statement-id>.<k>.`, so this is a search for that
/// shape among the names the walk has already collected: for each scope the reference can see,
/// every block *within* it that declares `name`, at the copy asked for.  Two blocks in one scope
/// declaring the same name make the index ambiguous, and an ambiguous name resolves to nothing
/// rather than to whichever came first.
fn copy_of(name: &str, k: usize, sc: &Scope, names: &BTreeSet<String>) -> Option<String> {
    let mut found: Option<String> = None;
    for p in &sc.prefixes {
        let from = format!("{p}#");
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
    // and the rest of the path is read against the copy it picks.  Only the root may carry one —
    // an index selects an instance of the block a name was declared in, and a field of one is
    // still a field.
    if let Some(Seg::Index(text)) = r.path.first() {
        let k = match value_of(text, &sc.vals, units) {
            Ok(v) if v.is_finite() && v >= 0.0 && v.round() == v => v as usize,
            _ => return None,
        };
        let abs = copy_of(&r.root.text, k, sc, names)?;
        let rest: Vec<String> = r.path[1..]
            .iter()
            .map(|s| match s {
                Seg::Field(f) => f.text.clone(),
                Seg::Index(t) => t.clone(),
            })
            .collect();
        // a second index would name a copy of a copy, which no statement makes
        if r.path[1..].iter().any(|s| matches!(s, Seg::Index(_))) {
            return None;
        }
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
            // in this order
            out.extend(d.membership.plane());
            out.extend(d.attitude.plane_ref());
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
            if let Some(r) = d.membership.plane_mut() {
                fix(r, bad);
            }
            if let Some(r) = d.attitude.plane_ref_mut() {
                fix(r, bad);
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
