//! Source-order statement expansion, repetition, seed settling and emission.

use super::*;

impl<'a> Walk<'a> {
    /// The view an enclosing instance is drawn `in`, put on one declaration its expansion
    /// makes (§6.7).  A datum or a curve is left alone — it has no points of its own to put
    /// there — and a declaration that already says which plane (a clause of its own, on a
    /// plane the component declares) may not be told twice.
    pub(super) fn stamp_scope_plane(&mut self, d: &mut Decl, scope: &Scope) {
        d.annotations.roles = d.annotations.roles.union(scope.in_roles);
        // the instance's classes, over the declaration's own: what the assembly says of an
        // instance is the later and stronger word, so a phantom's centreline is a phantom's
        for c in &scope.in_class.0 {
            if !d.class.has(c) {
                d.class.0.push(c.clone());
            }
        }
        let Some(p) = &scope.in_plane else { return };
        if !d.kind.bears_points() {
            return; // a plane's origin is the plane's, and a curve is its expressions
        }
        if !d.membership.join(&p.plane, crate::syntax::Source::Instance) {
            self.err(Code::E103, d.membership.span(), d.membership.cause().to_string());
        }
    }

    /// Walk one body, in source order.
    pub(super) fn body(
        &mut self,
        body: &[Stmt],
        scope: &Scope,
        vals: &mut BTreeMap<String, Aff>,
        path: &[PathStep],
        depth: usize,
    ) {
        if depth > MAX_DEPTH {
            if let Some(st) = body.first() {
                self.err(Code::E103, st.span, format!("nested more than {MAX_DEPTH} deep"));
            }
            return;
        }
        let prefix = scope.prefix().to_string();
        for st in body {
            let StmtKind::Group(g) = &st.kind else { continue };
            let name = &g.name.text;
            let conflicts = body.iter().any(|other| match &other.kind {
                StmtKind::Decl(d) => &d.name.key().text == name,
                StmtKind::Param(p) => &p.name.text == name,
                _ => false,
            }) || vals.contains_key(name) || scope.groups.contains(name)
                || self.aliases.iter().any(|(key, _, _)| key == &format!("{prefix}{name}"));
            if conflicts {
                self.err(Code::E001, g.name.span, format!("`{name}` is declared twice"));
            }
        }
        for st in body {
            if let StmtKind::Instance(inst) = &st.kind {
                let name = format!("{prefix}{}", inst.name.text);
                self.names.insert(name.clone());
                if inst.annotations.private { self.private_names.insert(name.clone(), scope.owner.clone()); }
                self.group_names.insert(name);
            }
        }
        // the root's numbers are the file's: the params of every module the file `use`s come
        // first, so the file's own may read them under their modules' paths
        if depth == 0 {
            let uses: Vec<String> = self.prog.uses.iter().map(|u| u.name.clone()).collect();
            for (k, v) in self.used_params(&uses) {
                vals.entry(k).or_insert(v);
            }
            // and those it imports bare, by their own names (§14.4 [0.48])
            for (k, v) in self.imported_params(None) {
                vals.entry(k).or_insert(v);
            }
        }
        // every `param` of the body, whatever line it stands on — a body is a set (P2)
        self.params(body, vals, scope);
        // and every instance's numbers, by its name (`ball.r`), over the body's own
        self.instance_numbers(body, vals, scope);
        // Remember ambient numbers so accidental capture gets an explicit diagnostic.
        if depth == 0 {
            self.file_vals = vals.clone();
        }
        // and with them the numbers in force are complete for every statement of the body:
        // the enclosing ones the caller passed and the body's own — so that is the table each
        // statement is emitted with, which is what an index (`p[n - 1]`) is read against.  The
        // root's scope arrived with an empty one, and a top-level index could read a literal
        // and not a `param` (#45.2).
        let mut groups = scope.groups.clone();
        groups.extend(body.iter().filter_map(|st| match &st.kind {
            StmtKind::Group(g) => Some(g.name.text.clone()),
            _ => None,
        }));
        let scope = &Scope { vals: vals.clone(), groups, ..scope.clone() };
        for st in body {
            if self.emitted() >= MAX_FLAT {
                self.err(
                    Code::E103,
                    st.span,
                    format!("more than {MAX_FLAT} statements once expanded"),
                );
                return;
            }
            match &st.kind {
                StmtKind::Chain(chain) => {
                    let mut chain = chain.clone();
                    let abs = format!("{prefix}{}", chain.name.key().text);
                    self.names.insert(abs.clone());
                    if chain.annotations.private { self.private_names.insert(abs.clone(), scope.owner.clone()); }
                    chain.annotations.roles = chain.annotations.roles.union(scope.in_roles);
                    chain.name = chain.name.prefixed(abs, scope.copies);
                    if scope.anonymous {
                        chain.name = crate::syntax::DeclName::Key(chain.name.key().clone());
                    }
                    self.emit(StmtKind::Chain(chain), st, scope, path);
                }
                StmtKind::Decl(d) => {
                    let abs = format!("{prefix}{}", d.name.key().text);
                    self.names.insert(abs.clone());
                    if d.annotations.private { self.private_names.insert(abs.clone(), scope.owner.clone()); }
                    let mut d2 = d.clone();
                    d2.name = d.name.prefixed(abs, scope.copies);
                    if scope.anonymous {
                        d2.name = crate::syntax::DeclName::Key(d2.name.key().clone());
                    }
                    // a computed point is made of expressions over the formals, which is a
                    // thing a curve can be and a drawing cannot: nothing on the sheet holds a
                    // point to a formula (§6.5)
                    if let Some([(x, xs), (y, ys)]) = &d.computed {
                        if self.sym.is_none() {
                            let n = &d.name.key().text;
                            self.err(
                                Code::E103,
                                d.name.span(),
                                format!(
                                    "`{n}` is a computed point, so its component is drawn only \
                                     as a curve: `e := Component(…).{n} over u in (a, b)`"
                                ),
                            );
                            continue;
                        }
                        d2.computed = Some([
                            (self.subst_sym(x, vals, scope), *xs),
                            (self.subst_sym(y, vals, scope), *ys),
                        ]);
                        self.emit(StmtKind::Decl(d2), st, scope, path);
                        continue;
                    }
                    if let Some(c) = d2.curve.as_mut() {
                        // the interval is written over the parameters in scope, like a number
                        for t in [&mut c.domain.0, &mut c.domain.1] {
                            match value_of(t, vals, self.units) {
                                Ok(v) => *t = crate::syntax::num(v),
                                Err(e) => self.err(Code::E103, st.span, format!("`{t}`: {e}")),
                            }
                        }
                        // an instance written in place is bound like any other and never
                        // drawn: the curve is the only thing made of it (§6.5)
                        if let CurveTarget::Anon(inst, point) = &c.target {
                            if let Some((_, _, _, key)) =
                                self.bind_instance(inst, scope, vals, false)
                            {
                                c.of = Some(crate::syntax::CurveOf {
                                    instance: key,
                                    point: written(point),
                                });
                            }
                        }
                    }
                    self.settle_seeds(&mut d2, vals, scope, st.span);
                    // and a solid's sweep, which is written in the same little language
                    if let Some(sw) = d2.sweep.as_mut() {
                        for a in sw.args_mut() {
                            self.settle_arg(a, vals, scope);
                        }
                    }
                    if let Some(e) = d2.angular_span.as_mut() {
                        self.settle_arg(&mut e.from, vals, scope);
                        self.settle_arg(&mut e.to, vals, scope);
                    }
                    if let Some(motion) = d2.motion.as_mut() {
                        for a in motion.args_mut() { self.settle_arg(a, vals, scope); }
                    }
                    self.stamp_scope_plane(&mut d2, scope);
                    self.emit(StmtKind::Decl(d2), st, scope, path);
                }
                StmtKind::Param(_) | StmtKind::Group(_) => {} // worked out above, before the walk
                StmtKind::Instance(inst) => {
                    let Some((comp, mut sub_vals, sides, key)) =
                        self.bind_instance(inst, scope, vals, true)
                    else {
                        continue;
                    };
                    // the instance's own `in`, or the one already in force around it — both at
                    // once is a plane given twice, which one statement may not do (§6.7)
                    let in_plane = match (inst.membership.plane(), &scope.in_plane) {
                        (Some(p), Some(_)) => {
                            self.err(Code::E103, p.span, inst.membership.cause().to_string());
                            scope.in_plane.clone()
                        }
                        // written here, in this scope: it resolves against these prefixes
                        (Some(p), None) => {
                            Some(InPlane { owner: scope.owner.clone(), plane: p.clone(), prefixes: scope.prefixes.clone(), closed: scope.closed })
                        }
                        (None, q) => q.clone(),
                    };
                    let forbidden = scope.vals.keys().chain(self.file_vals.keys()).cloned().collect();
                    let sc = Scope {
                        owner: key.clone(),
                        access: scope.access.clone(),
                        in_roles: scope.in_roles.union(inst.annotations.roles),
                        prefixes: vec![key],
                        closed: true,
                        forbidden,
                        groups: comp.formals.iter().filter(|f| f.ty == Ty::Group).map(|f| f.name.text.clone()).collect(),
                        cyc: None,
                        // a ring's copy reaches into the instances written in it
                        ring: scope.ring.clone(),
                        copies: scope.copies,
                        anonymous: scope.anonymous || inst.name.text.starts_with('#'),
                        vals: sub_vals.clone(),
                        in_plane,
                        in_class: {
                            let mut c = scope.in_class.clone();
                            c.0.extend(inst.class.0.iter().cloned());
                            c
                        },
                        // the sides this instance was given, and no others: a component reads a
                        // side by the name of its own formal, as it reads every other argument
                        sides,
                        // and its body's calls name components from the file it was written in
                        module: comp.module,
                    };
                    // a component reached again while it is still being expanded is a cycle:
                    // said once, at the call that closes it, and not walked
                    if let Some(at) = self.instantiating.iter().position(|c| std::ptr::eq(*c, comp)) {
                        let name = |c: &Component| c.name.as_ref().map_or(String::new(), |n| n.text.clone());
                        let through: Vec<String> =
                            self.instantiating[at + 1..].iter().map(|c| format!("`{}`", name(c))).collect();
                        let message = match through.is_empty() {
                            true => format!("`{}` instantiates itself", name(comp)),
                            false => format!("`{}` instantiates itself through {}", name(comp), through.join(", ")),
                        };
                        if !self.diagnostics.iter().any(|d| d.code == Code::E003 && d.span == st.span) {
                            self.err(Code::E003, st.span, message);
                        }
                        continue;
                    }
                    let mut instance_path = path.to_vec();
                    instance_path.push(PathStep::Instance(st.id));
                    self.instantiating.push(comp);
                    self.body(&comp.body, &sc, &mut sub_vals, &instance_path, depth + 1);
                    self.instantiating.pop();
                }
                StmtKind::Block(b) => {
                    if b.over.is_some() {
                        // how many is the chain's, and the chain may be a name no walk has
                        // reached yet (a forward reference, or one inside an instance written
                        // further down): expanded once every name is known, where the block
                        // itself stands in the output until then
                        self.pending.push(Pending {
                            st: st.clone(),
                            scope: scope.clone(),
                            vals: vals.clone(),
                            path: path.to_vec(),
                            depth,
                        });
                        self.out.push((st.clone(), path.to_vec(), scope.clone()));
                        continue;
                    }
                    let n = match value_of(&b.count, vals, self.units) {
                        Ok(v) if v.is_finite() && v >= 0.0 => v.round() as usize,
                        Ok(v) => {
                            self.err(
                                Code::E103,
                                b.span,
                                format!("`{}` is {v}, which is not a count", b.count),
                            );
                            continue;
                        }
                        Err(e) => {
                            self.err(Code::E103, b.span, format!("`{}`: {e}", b.count));
                            continue;
                        }
                    };
                    self.copies(b, st, scope, vals, path, depth, n, None);
                }
                // a constraint: its dimension is written in the component's own parameters, which
                // do not exist in the flat document, so they are worked out here
                // — and one written with a defined word is that word's body (§9.9)
                StmtKind::Relation(rel) => {
                    let Some(r2) = self.settle_relation(rel, vals, scope) else { continue };
                    self.emit(StmtKind::Relation(r2), st, scope, path);
                }
                // a claim over an interval: its numbers and its relations' worked out as above
                StmtKind::ClaimOver(c) => {
                    let mut c2 = c.clone();
                    for a in [&mut c2.from, &mut c2.to] { self.settle_arg(a, vals, scope); }
                    c2.body.retain_mut(|inner| {
                        let StmtKind::Relation(rel) = &inner.kind else { return true };
                        let Some(r2) = self.settle_relation(rel, vals, scope) else { return false };
                        inner.kind = StmtKind::Relation(r2);
                        true
                    });
                    self.emit(StmtKind::ClaimOver(c2), st, scope, path);
                }
                // a gauge or an orientation: kept as written, resolved later
                other => self.emit(other.clone(), st, scope, path),
            }
        }
    }

    /// A relation's numbers worked out against the parameters in scope, in either of its
    /// representations, and the enclosing instances' classes stamped over its own — the relation
    /// it states, a defined word's body expanded (`expand_word`; `None` where it cannot be).
    fn settle_relation(&mut self, rel: &crate::syntax::Relation, vals: &BTreeMap<String, Aff>, scope: &Scope)
        -> Option<crate::syntax::Relation> {
        let mut r2 = self.expand_word(rel, scope)?;
        if let Some(w) = r2.form.written_mut() {
            for a in w.args.iter_mut() {
                match a {
                    crate::syntax::OpArg::Slot { arg, .. } => self.settle_arg(arg, vals, scope),
                    crate::syntax::OpArg::Vector { parts, .. } => {
                        for arg in parts {
                            self.settle_arg(arg, vals, scope)
                        }
                    }
                    crate::syntax::OpArg::Named(_, arg) => self.settle_arg(arg, vals, scope),
                    crate::syntax::OpArg::Dim(text, span) => {
                        self.settle_dim(text, *span, vals, scope)
                    }
                    crate::syntax::OpArg::Ent(_) => {}
                }
            }
        }
        for a in r2.form.canonical_args_mut().iter_mut().flatten() {
            self.settle_arg(a, vals, scope);
        }
        // the enclosing instances' classes, over the statement's own: a ghosted part's
        // dimensions are the ghost's (`display: geometry`) as its lines are
        for c in &scope.in_class.0 {
            if !r2.class.has(c) {
                r2.class.0.push(c.clone());
            }
        }
        Some(r2)
    }

    /// A seed written as an expression is worked out here, against the parameters in scope,
    /// and is a number from now on.
    fn settle_seeds(
        &mut self,
        d: &mut Decl,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
        span: Span,
    ) {
        // An unbound numeric formal is a solver unknown, but a hint still needs a
        // starting value. Read its provisional zero only in hints; constraints keep
        // the symbolic value. Traced components retain their parameterized seeds.
        let provisional;
        let vals = if self.sym.is_none() && vals.values().any(|v| v.free.is_some()) {
            provisional = vals.iter().map(|(name, value)| {
                (name.clone(), Aff::of_dim(value.c, value.dim))
            }).collect();
            &provisional
        } else {
            vals
        };
        // a rational spline's weights: numbers over this scope's values, dimensionless
        for w in d.weights.iter_mut().flatten() {
            let Some(t) = w.text.take() else { continue };
            let t = self.subst_sym(&t, vals, scope);
            match super::values::value_aff(&t, vals, self.units) {
                Ok(a) if a.free.is_none() && a.dim.is_scalar() => w.value = a.c,
                Ok(_) => self.err(Code::E103, w.span, format!("a weight is a plain number, and `{t}` is not")),
                Err(e) => self.err(Code::E103, w.span, format!("`{t}`: {e}")),
            }
        }
        for i in 0..d.seed_text.len() {
            let Some(t) = d.seed_text[i].take() else { continue };
            if let Some(v) = self.settle_seed(t, &mut d.seed_text[i], vals, scope, span) {
                d.seed[i] = v;
            }
        }
        // a seed in a child slot — `line l(hint((w / 2, 0)), …)` — is the same clause one
        // level down, and is written over the same parameters
        for kid in d.children.iter_mut().flatten() {
            let crate::syntax::Kid::Hint(ks) = kid else { continue };
            for i in 0..3 {
                let Some(t) = ks.text[i].take() else { continue };
                if let Some(v) = self.settle_seed(t, &mut ks.text[i], vals, scope, span) {
                    ks.v[i] = v;
                }
            }
        }
        for (b, _) in d.seed_at.iter_mut().flat_map(|a| a.texts_mut()) {
            // a place's numbers are texts over the parameters in scope, read later — by the
            // trace compile off its variable table, or by the build off the geometry's seeds —
            // so the numbers in force are written in and the rest is left for that reader
            *b = if self.sym.is_some() {
                self.subst_sym(b, vals, scope)
            } else {
                substitute(b, vals, self.units)
            };
        }
    }

    /// One seed's text, settled over the parameters in scope: the number it comes to, or —
    /// where it cannot come to one here — the text kept in `keep` for a later reader.
    ///
    /// Two readers keep a text.  A trace block's compile reads a variable of the curve or a
    /// formal's coordinate off its own table.  And on the sheet a seed may **read geometry**
    /// (`hint((k.center.x + k.r, pin.y))`): the value read is that scalar's own *seed*,
    /// which is a fact about the built drawing and not about this scope, so `program::build`
    /// works it out once every declaration has one (§6.4).  Any other failure is the error it
    /// always was.
    fn settle_seed(
        &mut self,
        t: String,
        keep: &mut Option<String>,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
        span: Span,
    ) -> Option<f64> {
        let t = self.subst_sym(&t, vals, scope);
        match value_of(&t, vals, self.units) {
            Ok(v) => Some(v),
            Err(_) if self.sym.is_some() => {
                *keep = Some(t);
                None
            }
            // the numbers in force written in, so what the build reads is geometry and nothing
            // of this scope's
            Err(_) if reads_geometry(&t, self.units) => {
                *keep = Some(substitute(&t, vals, self.units));
                None
            }
            Err(e) => {
                self.err(Code::E103, span, format!("`{t}`: {e}"));
                None
            }
        }
    }

    /// The copies of one block: `n` of them, each its body under the block's prefix, with the
    /// trailing joint stated between neighbours.  Where the block runs over a chain, `edges`
    /// holds each copy's edge — the link as the chain wrote it and the scope it was written in —
    /// and copy `k` gets an alias `<block>#<id>.<k>.<var>` for it, which is how `e` is found
    /// through the copy's prefix like any name, and passed on to an instance as any alias is.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn copies(
        &mut self,
        b: &Block,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
        depth: usize,
        n: usize,
        edges: Option<(&Name, &[(Ref, Scope)])>,
    ) {
        // a block's index and its edge binder are names of each copy, and one over a name the
        // enclosing scope already has would read two ways in one body (E002): said once per block
        let binders = b.binder.iter().chain(edges.map(|(v, _)| v));
        for name in binders {
            let taken = vals.contains_key(&name.text)
                || scope.groups.contains(&name.text)
                || self.names.contains(&format!("{}{}", scope.prefix(), name.text));
            if taken && !self.diagnostics.iter().any(|d| d.code == Code::E002 && d.span == name.span) {
                self.err(
                    Code::E002,
                    name.span,
                    format!("`{}` is already a name here, and a block's binder may not shadow it", name.text),
                );
            }
        }
        if n == 0 {
            return;
        }
        if n > MAX_FLAT {
            self.err(Code::E103, b.span, format!("{n} copies is more than {MAX_FLAT}"));
            return;
        }
        let block_prefix = format!("{}#{}.", scope.prefix(), st.id.0);
        // a ring: copies the representative's turns.  One inside another is refused (E022) and
        // expanded as the cycle it is besides, so what it makes is still there to be named; and
        // its index, read anywhere in its body, varies nothing (E015)
        let ring = match (b.kind, &b.about) {
            (BlockKind::Ring, Some(_)) if scope.ring.is_some() => {
                self.once(Code::E022, st.span, "a `ring` inside a `ring` is not supported: turn the \
                    inner one's copies about a centre the outer ring does not move, outside it");
                None
            }
            (BlockKind::Ring, Some(about)) => {
                let i = b.binder.as_ref().map_or("", |n| n.text.as_str());
                for &s in &b.index_reads {
                    self.once(Code::E015, s, format!("`{i}` is a `ring`'s index: every copy is the \
                        first turned, so there is nothing for it to vary — reach a neighbour by \
                        `next` or `prev`"));
                }
                self.rings.push((block_prefix.clone(), about.clone(), scope.clone(), n, st.span));
                Some(block_prefix.clone())
            }
            _ => None,
        };
        let mut ranges: Vec<(usize, usize)> = Vec::new();
        for k in 0..n {
            let mut sub = vals.clone();
            if let Some(i) = &b.binder {
                sub.insert(i.text.clone(), Aff::num(k as f64));
            }
            if let Some((var, links)) = edges {
                let (r, sc) = &links[k];
                let key = format!("{block_prefix}{k}.{}", var.text);
                self.aliases.push((key, r.clone(), sc.clone()));
            }
            let sc = Scope {
                owner: scope.owner.clone(),
                access: scope.access.clone(),
                in_roles: scope.in_roles,
                prefixes: std::iter::once(format!("{block_prefix}{k}."))
                    .chain(scope.prefixes.iter().cloned())
                    .collect(),
                // `next` and `prev` mean something only where the copies close
                closed: scope.closed,
                forbidden: scope.forbidden.clone(),
                groups: scope.groups.clone(),
                cyc: b.kind.wraps().then(|| Cyc { prefix: block_prefix.clone(), k, n }),
                ring: match &ring {
                    Some(prefix) => Some(super::Ring { prefix: prefix.clone(), turned: k > 0 }),
                    None => scope.ring.clone(),
                },
                // the prefix just built is the block's id, so every declaration
                // below is a copy, however deep and through however many instances
                copies: true,
                anonymous: scope.anonymous,
                vals: sub.clone(),
                in_plane: scope.in_plane.clone(),
                in_class: scope.in_class.clone(),
                sides: scope.sides.clone(),
                module: scope.module,
            };
            let mut p2 = path.to_vec();
            p2.push(PathStep::Copy { block: st.id, index: k as u32 });
            let from = self.out.len();
            self.body(&b.body, &sc, &mut sub, &p2, depth + 1);
            // the trailing joint's relations, stated between this copy and the
            // next: every copy for a cycle (the wrap seals the loop),
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

    /// How many statements the walk has made, counting those held aside while a deferred block
    /// is expanded into a vector of its own (`expand_pending`).
    pub(super) fn emitted(&self) -> usize {
        self.out.len() + self.held
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
    fn emit(&mut self, kind: StmtKind, st: &Stmt, scope: &Scope, path: &[PathStep]) {
        let stmt = Stmt { id: st.id, kind, span: st.span, chained: st.chained };
        self.out.push((stmt, path.to_vec(), scope.clone()));
    }
}
