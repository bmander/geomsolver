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
use crate::syntax::{
    BlockKind, Component, Decl, Name, Program, Ref, Seg, Span, Stmt, StmtKind,
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

/// What the names in one statement are resolved against: the prefixes it is nested in, innermost
/// first, and the aliases the instantiation bound.
#[derive(Clone, Default)]
struct Scope {
    prefixes: Vec<String>,
    binds: BTreeMap<String, String>,
    cyc: Option<Cyc>,
    /// The numbers in force where the statement was written — the enclosing counts, params and
    /// block binders.  An index (`p[i + 1]`) is an expression over exactly these, and references
    /// are resolved in a later pass where the walk's own environment is gone, so it travels here.
    vals: BTreeMap<String, Aff>,
}

pub struct Expansion {
    pub flat: Vec<Flat>,
    pub errors: Vec<(Span, String)>,
}

struct Walk<'a> {
    prog: &'a Program,
    out: Vec<(Stmt, Vec<u32>, Scope)>,
    /// Every absolute name a declaration will make.  Collected as the walk goes and used to
    /// resolve references afterwards, so forward reference works — which spec P2 requires, since
    /// a body is a set and a set has no "before".
    names: BTreeSet<String>,
    /// `port x = y`: one name for what another names.  Resolved transitively, after the walk.
    aliases: Vec<(String, Ref, Scope)>,
    errors: Vec<(Span, String)>,
}

/// Expand a program's root component into a flat list of declarations, constraints, gauges and
/// orientations, with every name made absolute.
pub fn expand(prog: &Program) -> Expansion {
    let mut w = Walk {
        prog,
        out: Vec::new(),
        names: BTreeSet::new(),
        aliases: Vec::new(),
        errors: Vec::new(),
    };
    let root = prog.root();
    let scope = Scope { prefixes: vec![String::new()], ..Scope::default() };
    let mut vals: BTreeMap<String, Aff> = BTreeMap::new();
    w.body(&root.body, &scope, &mut vals, &[], 0);
    let flat = w.resolve();
    Expansion { flat, errors: w.errors }
}

impl<'a> Walk<'a> {
    fn err(&mut self, span: Span, msg: impl Into<String>) {
        if self.errors.len() < 200 {
            self.errors.push((span, msg.into()));
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
        for st in body {
            if self.out.len() >= MAX_FLAT {
                self.err(st.span, format!("more than {MAX_FLAT} statements once expanded"));
                return;
            }
            match &st.kind {
                StmtKind::Decl(d) => {
                    let abs = format!("{prefix}{}", d.name.text);
                    self.names.insert(abs.clone());
                    let mut d2 = d.clone();
                    d2.name = Name { text: abs, span: d.name.span };
                    // a curve's numbers and the interval it is drawn over are written over the
                    // parameters in scope too, and are worked out in the same pass
                    for (_, t) in d2.values.iter_mut() {
                        match value_of(t, vals) {
                            Ok(v) => *t = crate::syntax::num(v),
                            Err(e) => self.err(st.span, format!("`{t}`: {e}")),
                        }
                    }
                    if let Some((a, b)) = d2.domain.as_mut() {
                        for t in [a, b] {
                            match value_of(t, vals) {
                                Ok(v) => *t = crate::syntax::num(v),
                                Err(e) => self.err(st.span, format!("`{t}`: {e}")),
                            }
                        }
                    }
                    // a seed written as an expression is worked out here, against the parameters
                    // in scope, and is a number from now on
                    for i in 0..d2.seed_text.len() {
                        let Some(t) = d2.seed_text[i].clone() else { continue };
                        match value_of(&t, vals) {
                            Ok(v) => d2.seed[i] = v,
                            Err(e) => self.err(st.span, format!("`{t}`: {e}")),
                        }
                        d2.seed_text[i] = None;
                    }
                    self.emit(StmtKind::Decl(d2), st, scope, path);
                }
                // `port lead: Point` is a fresh declaration that the boundary also names.  There
                // is nothing else to it: a port carries no joint, no direction and no constraint.
                StmtKind::Port(p) => {
                    if let Some(kind) = p.declare {
                        let abs = format!("{prefix}{}", p.name.text);
                        self.names.insert(abs.clone());
                        let d = Decl {
                            kind,
                            name: Name { text: abs, span: p.name.span },
                            children: vec![Vec::new(); count_children(kind)],
                            seed: vec![0.0; count_scalars(kind)],
                            seed_text: vec![None; count_scalars(kind)],
                            seed_spans: vec![Span::default(); count_scalars(kind)],
                            hint_span: None,
                            knots: None,
                            def: None,
                            values: Vec::new(),
                            domain: None,
                            class: Default::default(),
                            class_span: Span::default(),
                            seed_at: None,
                        };
                        self.emit(StmtKind::Decl(d), st, scope, path);
                    } else if let Some(r) = &p.alias {
                        let abs = format!("{prefix}{}", p.name.text);
                        self.aliases.push((abs, r.clone(), scope.clone()));
                    }
                }
                StmtKind::Param(pd) => {
                    match value_of(&pd.text, vals) {
                        Ok(v) => {
                            vals.insert(pd.name.text.clone(), Aff::num(v));
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
                    let (binds, mut sub_vals) = self.bind(&comp, inst, scope, vals);
                    let mut sc = Scope {
                        prefixes: std::iter::once(format!("{prefix}{}.", inst.name.text))
                            .chain(scope.prefixes.iter().cloned())
                            .collect(),
                        binds,
                        cyc: None,
                        vals: sub_vals.clone(),
                    };
                    sc.cyc = scope.cyc.clone();
                    self.body(&comp.body, &sc, &mut sub_vals, path, depth + 1);
                }
                StmtKind::Block(b) => {
                    let n = match value_of(&b.count, vals) {
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
                    for k in 0..n {
                        let mut sub = vals.clone();
                        if let Some(i) = &b.binder {
                            sub.insert(i.text.clone(), Aff::num(k as f64));
                        }
                        let sc = Scope {
                            prefixes: std::iter::once(format!("{block_prefix}{k}."))
                                .chain(scope.prefixes.iter().cloned())
                                .collect(),
                            binds: scope.binds.clone(),
                            // `next` and `prev` mean something only where the copies close
                            cyc: (b.kind != BlockKind::Repeat).then(|| Cyc {
                                prefix: block_prefix.clone(),
                                k,
                                n,
                            }),
                            vals: sub.clone(),
                        };
                        let mut p2 = path.to_vec();
                        p2.push(k as u32);
                        self.body(&b.body, &sc, &mut sub, &p2, depth + 1);
                    }
                }
                // a constraint: its dimension is written in the component's own parameters, which
                // do not exist in the flat document, so they are worked out here
                StmtKind::Relation(rel) => {
                    let mut r2 = rel.clone();
                    for a in r2.args.iter_mut().flatten() {
                        match a {
                            crate::syntax::Arg::Dim { text, span } => match settle(text, vals) {
                                Ok(t) => *text = t,
                                Err(e) => self.err(*span, format!("`{text}`: {e}")),
                            },
                            // a contact's seed may be written over the parameters in scope too
                            crate::syntax::Arg::SeedExpr { text, pinned, span } => {
                                match value_of(text, vals) {
                                    Ok(v) => {
                                        *a = crate::syntax::Arg::Seed {
                                            value: v,
                                            pinned: *pinned,
                                        }
                                    }
                                    Err(e) => self.err(*span, format!("`{text}`: {e}")),
                                }
                            }
                            _ => {}
                        }
                    }
                    self.emit(StmtKind::Relation(r2), st, scope, path);
                }
                // a gauge or an orientation: kept as written, resolved later
                other => self.emit(other.clone(), st, scope, path),
            }
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

    /// Bind an instantiation's arguments to the component's formals.
    ///
    /// An entity argument *aliases*: the formal and the actual denote one entity, at no cost.  A
    /// value argument is worked out here and is a number from then on.
    fn bind(
        &mut self,
        comp: &Component,
        inst: &crate::syntax::Instance,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
    ) -> (BTreeMap<String, String>, BTreeMap<String, Aff>) {
        use crate::syntax::{InstVal, Ty};
        let mut binds = BTreeMap::new();
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
                    binds.insert(f.name.text.clone(), String::new());
                    self.aliases.push((
                        format!("\u{1}{}\u{1}{}", inst.name.text, f.name.text),
                        r.clone(),
                        scope.clone(),
                    ));
                }
                (Ty::Ent(_), InstVal::Expr(t)) => {
                    self.err(a.span, format!("`{}` wants an entity, and `{t}` is a number", f.name.text))
                }
                (_, InstVal::Expr(t)) => match value_of(t, vals) {
                    Ok(v) => {
                        sub.insert(f.name.text.clone(), Aff::num(v));
                    }
                    Err(e) => self.err(a.span, format!("`{}`: {e}", f.name.text)),
                },
                (_, InstVal::Ref(r)) => match value_of(&r.root.text, vals) {
                    Ok(v) => {
                        sub.insert(f.name.text.clone(), Aff::num(v));
                    }
                    Err(e) => self.err(a.span, format!("`{}`: {e}", f.name.text)),
                },
            }
        }
        (binds, sub)
    }

    /// Turn every reference into the absolute name of what it denotes.
    fn resolve(&mut self) -> Vec<Flat> {
        // port aliases first, and transitively: `port a = b` where `b` is itself a port
        let mut alias: BTreeMap<String, String> = BTreeMap::new();
        for (abs, r, sc) in self.aliases.clone() {
            if let Some((target, _)) = lookup(&r, &sc, &self.names, &alias) {
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
        let mut flat = Vec::with_capacity(out.len());
        for (mut st, path, mut sc) in out {
            // an instance's entity arguments: the formal now names what the caller passed
            for (formal, target) in sc.binds.iter_mut() {
                if target.is_empty() {
                    // the instance's own prefix is the innermost; the key was stashed under the
                    // instance name so several instances of one component do not collide
                    let inner = sc.prefixes.first().cloned().unwrap_or_default();
                    let iname = inner.trim_end_matches('.').rsplit('.').next().unwrap_or("");
                    if let Some(t) = alias.get(&format!("\u{1}{iname}\u{1}{formal}")) {
                        *target = t.clone();
                    }
                }
            }
            let mut bad: Vec<(Span, String)> = Vec::new();
            rewrite(&mut st.kind, &sc, &self.names, &alias, &mut bad);
            for (span, msg) in bad {
                self.err(span, msg);
            }
            flat.push(Flat { stmt: st, path });
        }
        flat
    }
}

fn count_children(k: EntKind) -> usize {
    k.fields().iter().filter(|(_, f)| *f != crate::model::Field::Scalar).count()
}

fn count_scalars(k: EntKind) -> usize {
    k.fields().iter().filter(|(_, f)| *f == crate::model::Field::Scalar).count()
}

/// A number worked out while elaborating.
///
/// The language's angle is the degree — `sin(90)` is 1, and every dimension a person reads is in
/// degrees — so a whole turn is `tau = 360`, not 2π.  Keeping `tau` at 2π and the texts in degrees
/// would give two units one name.
fn value_of(text: &str, env: &BTreeMap<String, Aff>) -> Result<f64, String> {
    let t = text.trim();
    if t.is_empty() {
        return Err("nothing to work out".to_string());
    }
    let mut env = env.clone();
    env.entry("tau".to_string()).or_insert_with(|| Aff::num(360.0));
    env.entry("turn".to_string()).or_insert_with(|| Aff::num(360.0));
    let p = expr::parse(t)?;
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
fn settle(text: &str, vals: &BTreeMap<String, Aff>) -> Result<String, String> {
    let sub = substitute(text, vals);
    // nothing of the component's in it: leave it exactly as written, so `h = w / 2` and `3 1/8`
    // reach the document with the form somebody typed
    if sub == text {
        return Ok(text.to_string());
    }
    let p = expr::parse(&sub)?;
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
            let known = vals
                .get(&word)
                .and_then(|a| a.number())
                .or_else(|| (word == "tau" || word == "turn").then_some(360.0));
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
) -> Option<(String, Vec<String>)> {
    // `p[k]` names *which copy* of a repeated statement, so it is resolved before anything else
    // and the rest of the path is read against the copy it picks.  Only the root may carry one —
    // an index selects an instance of the block a name was declared in, and a field of one is
    // still a field.
    if let Some(Seg::Index(text)) = r.path.first() {
        let k = match value_of(text, &sc.vals) {
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
        if let Some(t) = sc.binds.get(&cand) {
            if !t.is_empty() {
                return Some((t.clone(), rest));
            }
        }
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
    bad: &mut Vec<(Span, String)>,
) {
    let fix = |r: &mut Ref, bad: &mut Vec<(Span, String)>| match lookup(r, sc, names, alias) {
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
        }
        StmtKind::Relation(rel) => {
            for a in rel.args.iter_mut().flatten() {
                if let crate::syntax::Arg::Ref(r) = a {
                    fix(r, bad);
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
