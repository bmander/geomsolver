//! Component argument binding, instance validation and module parameter environments.

use super::*;

impl<'a> Walk<'a> {
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
    pub(super) fn weld(
        &mut self,
        j: &OpenJoint,
        kind: BlockKind,
        block_prefix: &str,
        n: usize,
        ranges: &[(usize, usize)],
    ) {
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
    pub(super) fn fill(
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
    pub(super) fn bind_instance(
        &mut self,
        inst: &crate::syntax::Instance,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        drawn: bool,
    ) -> Option<(&'a Component, BTreeMap<String, Aff>, BTreeMap<String, String>, String)> {
        let index = match self.prog.resolve_component(&inst.component.text, scope.module) {
            Ok(i) => i,
            Err(m) => {
                self.err(Code::E103, inst.component.span, m);
                return None;
            }
        };
        let comp = &self.prog.components[index];
        if self.called.insert(inst.span) {
            self.check_call(comp, inst);
        }
        let (sub_vals, sides) = self.bind(comp, inst, scope, vals);
        let key = format!("{}{}.", scope.prefix(), inst.name.text);
        self.instances.push(InstanceInfo {
            prefix: key.clone(),
            component: inst.component.text.clone(),
            comp: index,
            ents: comp
                .formals
                .iter()
                .filter(|f| matches!(f.ty, Ty::Ent(_)))
                .map(|f| (f.name.text.clone(), None))
                .collect(),
            values: sub_vals.clone(),
            drawn,
        });
        Some((comp, sub_vals, sides, key))
    }

    /// Refuse an argument a long formal list would silently take for another (§4.1).
    ///
    /// Positional binding is a count, and a count is exactly what a reader of a call cannot see:
    /// four times in one drawing an argument written after a labelled one landed on the formal
    /// beside the one it was meant for, and each time what came back was a complaint about
    /// something else — `views` is not a number, `n` is Scalar and this is Angle, a hexagon whose
    /// `phase` had arrived as its side count.  So the count is allowed only where the eye can
    /// check it: **the entities, in order, before every label**, which is how an instance names
    /// what it is written over — and every *number* carries the formal's name, which is the half
    /// a long list gets wrong.  Neither refusal is about a value, so both are asked of the text
    /// once, before anything is bound.
    fn check_call(&mut self, comp: &Component, inst: &crate::syntax::Instance) {
        use crate::syntax::Ty;
        let mut labelled: Option<&str> = None;
        let mut positional = 0usize;
        let mut given = BTreeSet::new();
        for a in &inst.args {
            let formal_name = a.label.as_ref().map(|l| l.text.as_str())
                .or_else(|| comp.formals.get(positional).map(|f| f.name.text.as_str()));
            if let Some(name) = formal_name.filter(|_| labelled.is_none() || a.label.is_some()) {
                if !given.insert(name) {
                    self.err(Code::E103, a.span, format!("argument `{name}` is given twice"));
                }
            }
            if let Some(l) = &a.label {
                labelled.get_or_insert(l.text.as_str());
                continue;
            }
            let formal = comp.formals.get(positional);
            positional += 1;
            // past the end of the list is `bind`'s to report — it is no such parameter, whatever
            // the argument is written as
            let said = if let Some(prev) = labelled {
                format!(
                    "a positional argument after `{prev}:` — an instance gives the entities by \
                     position, in order, and everything past the first label carries one too \
                     (§4.1)"
                )
            } else if let Some(f) = formal.filter(|f| !matches!(f.ty, Ty::Ent(_) | Ty::Group)) {
                format!(
                    "`{0}` is a number, and a number is given by label: write `{0}: …` (§4.1)",
                    f.name.text
                )
            } else {
                continue;
            };
            self.err(Code::E004, a.span, said);
        }
    }

    /// Bind an instantiation's arguments to the component's formals.
    ///
    /// An entity argument *aliases*: the formal and the actual denote one entity, at no cost.  A
    /// value argument is worked out here and is a number from then on.
    ///
    /// The alias is recorded under the instance's **absolute** prefix, `{prefix}{inst}.{formal}`,
    /// resolved in the caller's scope.  Absolute, because that is the one key every reader already reaches —
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
    ) -> (BTreeMap<String, Aff>, BTreeMap<String, String>) {
        use crate::syntax::{InstVal, Ty};
        let prefix = scope.prefix().to_string();
        let mut sub: BTreeMap<String, Aff> = BTreeMap::new();
        let mut sides: BTreeMap<String, String> = BTreeMap::new();
        for (formal, a) in formals_of(comp, inst) {
            let Some(f) = formal else {
                self.err(
                    Code::E103,
                    a.span,
                    format!("`{}` has no such parameter", inst.component.text),
                );
                continue;
            };
            match (&f.ty, &a.value) {
                // braces are a definition's, or a member's of one: a call is given a name
                (_, InstVal::Group(_)) => self.err(Code::E103, a.span, format!(
                    "a group is written in braces only where it is defined: `{n} := {{…}}`, \
                     then `{n}: {n}`", n = f.name.text)),
                (Ty::Group, InstVal::Ref(r)) => {
                    let actual = written(r);
                    let key = formal_name(&prefix, &inst.name.text, &f.name.text);
                    self.aliases.push((key.clone(), r.clone(), scope.clone()));
                    self.group_bindings.push((key, a.span));
                    let start = format!("{actual}.");
                    for (name, value) in vals {
                        if let Some(member) = name.strip_prefix(&start) {
                            sub.insert(format!("{}.{member}", f.name.text), value.clone());
                        }
                    }
                }
                (Ty::Group | Ty::Ent(_), InstVal::Hint(..)) => self.err(Code::E103, a.span,
                    format!("`{}` is not a number, so there is nothing to seed", f.name.text)),
                // `beta: hint(15deg)` — the formal stays unbound, an unknown of the drawing like
                // any unbound numeric formal (below), and its solve begins at the number
                (ty @ (Ty::Int | Ty::Scalar | Ty::Length | Ty::Angle), InstVal::Hint(t, _)) => {
                    let name = formal_name(&prefix, &inst.name.text, &f.name.text);
                    match self.seed_number(t, *ty, vals) {
                        Ok(v) if *ty != Ty::Int => {
                            let declared = crate::model::Declared { dim: ty.dim(), seed: Some(v) };
                            self.unknowns.insert(name, declared);
                        }
                        Ok(_) => self.err(Code::E103, a.span,
                            format!("`{}` is a count, which is never an unknown", f.name.text)),
                        Err(e) => self.err(Code::E103, a.span, format!("`{t}`: {e}")),
                    }
                }
                (Ty::Group, InstVal::Expr(t)) => self.err(Code::E103, a.span,
                    format!("`{}` wants a group or layout instance, not `{t}`", f.name.text)),
                (Ty::Ent(_), InstVal::Ref(r)) => {
                    // recorded unresolved; the resolve pass turns it into an absolute name in the
                    // *caller's* scope, which is what makes it an alias rather than a copy
                    self.aliases.push((
                        formal_name(&prefix, &inst.name.text, &f.name.text),
                        r.clone(),
                        scope.clone(),
                    ));
                }
                (Ty::Ent(_), InstVal::Expr(t)) => self.err(
                    Code::E103,
                    a.span,
                    format!("`{}` wants an entity, and `{t}` is a number", f.name.text),
                ),
                // **a side is a word** (§9.2): `left`, `right`, or the name of a side the caller
                // was given itself, which is how one is passed down a chain of components
                (Ty::Side, v) => {
                    let w = match v {
                        InstVal::Ref(r) if r.path.is_empty() => r.root.text.clone(),
                        InstVal::Ref(r) => r.root.text.clone(),
                        InstVal::Expr(t) | InstVal::Hint(t, _) => t.clone(),
                        InstVal::Group(_) => unreachable!("refused by the first arm"),
                    };
                    let w = scope.sides.get(&w).cloned().unwrap_or(w);
                    if w == "left" || w == "right" {
                        sides.insert(f.name.text.clone(), w);
                    } else {
                        let m =
                            format!("`{}` is a side: `left` or `right`, not `{w}`", f.name.text);
                        self.err(Code::E103, a.span, m);
                    }
                }
                // A formal *declares* what its argument is — `phi: Angle`, `m: Length` — so the
                // number it stands for carries that dimension through the component's body.
                // This is where `x := w + phi` is caught: nothing else in a component says
                // what a number is, and the substitution `settle` performs erases it.
                (ty, InstVal::Expr(t)) => {
                    self.bind_value(&mut sub, f, *ty, t, vals, scope, &inst.name.text, a.span)
                }
                (ty, InstVal::Ref(r)) => {
                    let t = written(r);
                    self.bind_value(&mut sub, f, *ty, &t, vals, scope, &inst.name.text, a.span)
                }
            }
        }
        // A numeric formal the instance leaves unbound is an **unknown of the drawing**, as an
        // input with no value is at the top of a document — a leg drawn with its crank angle
        // unbound has a crank that turns.  Named under the instance's own prefix (`leg.theta`), so
        // two instances that leave the same formal unbound have two unknowns and not one shared one
        // — and inside a traced component the name is no column of the curve, which is how a nested
        // instance's unbound formal is reported rather than captured by an outer one's.
        for f in &comp.formals {
            if f.ty == Ty::Group && !inst.args.iter().enumerate().any(|(i, a)|
                a.label.as_ref().map_or_else(|| comp.formals.get(i).is_some_and(|p| p.name.text == f.name.text), |l| l.text == f.name.text)) {
                self.err(Code::E103, inst.span, format!("missing group argument `{}`", f.name.text));
            }
            // a side left unbound is not an unknown of the drawing: the *statement* it reaches
            // says nothing about which side, which is the magnitude form and a solution set of
            // both — so it is left out of the table and the body writes no side at all
            if !f.ty.number() || sub.contains_key(&f.name.text) {
                continue;
            }
            let name = formal_name(&prefix, &inst.name.text, &f.name.text);
            // one table of every unknown, seeded by the call's `hint(…)` where it wrote one
            let unknown = crate::model::Declared { dim: f.ty.dim(), seed: None };
            self.unknowns.entry(name.clone()).or_insert(unknown);
            sub.insert(f.name.text.clone(), free(name, f.ty));
        }
        (sub, sides)
    }

    /// A module's top-level `param`s, worked out once — in the module's own scope, by their own
    /// names, which has no numbers but its own and those of the modules it `use`s.  Its groups
    /// are known under the module's path (`components.dims.vtwin_dims`), the name a file passes.
    pub(super) fn module_params(&mut self, k: usize) -> BTreeMap<String, Aff> {
        if let Some(Some(v)) = self.module_vals.get(k) {
            return v.clone();
        }
        // marked done before the walk, so a module reached again through its own `use`s reads
        // nothing from itself rather than recursing
        if let Some(slot) = self.module_vals.get_mut(k) {
            *slot = Some(BTreeMap::new());
        }
        let m = &self.prog.modules[k];
        let scope = Scope { prefixes: vec![format!("{}.", m.name)], module: Some(k), ..Scope::default() };
        let mut vals: BTreeMap<String, Aff> = BTreeMap::new();
        let (body, uses) = (m.root.body.clone(), m.uses.iter().map(|u| u.name.clone()).collect::<Vec<_>>());
        // the modules it uses first, under their paths, so its own params may read them
        let mut used = self.used_params(&uses);
        used.extend(self.imported_params(Some(k)));
        for (name, v) in &used {
            vals.insert(name.clone(), v.clone());
        }
        self.params(&body, &mut vals, &scope);
        // and only its own go out: a file reaches another module's numbers through a `use` of
        // its own, so passing these on would only lengthen their names at every level
        vals.retain(|name, _| !used.contains_key(name));
        if let Some(slot) = self.module_vals.get_mut(k) {
            *slot = Some(vals.clone());
        }
        vals
    }

    /// **The values and groups a file imports bare** (§14.4 [0.48]): `use components.dims
    /// (vtwin_dims)` reads `vtwin_dims` and its members as the module's, beside their full path.
    /// A group so imported is an alias of the module's, so a call is handed it by either name.
    pub(super) fn imported_params(&mut self, from: Option<usize>) -> BTreeMap<String, Aff> {
        let mut out: BTreeMap<String, Aff> = BTreeMap::new();
        let prog = self.prog;
        for u in prog.use_stmts(from).iter().filter(|u| !u.names.is_empty()) {
            let module = &u.name;
            let Some(k) = prog.module_named(module) else { continue };
            let params = self.module_params(k);
            for n in u.names.iter().map(|n| &n.text) {
                let member = format!("{n}.");
                for (key, v) in &params {
                    if key == n || key.starts_with(&member) {
                        out.insert(key.clone(), v.clone());
                    }
                }
                let full = format!("{module}.{n}");
                if from.is_none() && self.group_names.contains(&full) {
                    let root = Scope { prefixes: vec![String::new()], ..Scope::default() };
                    self.aliases.push((n.clone(), Ref::new(full), root));
                }
            }
        }
        out
    }

    /// The params of the modules a file names in its `use`s, each under the module's full path
    /// (`hardware.nut14_af`, `components.dims.vtwin_dims.bore`): nothing a module defines is read
    /// bare outside it (§14.4).
    pub(super) fn used_params(&mut self, uses: &[String]) -> BTreeMap<String, Aff> {
        let mut out: BTreeMap<String, Aff> = BTreeMap::new();
        for name in uses {
            let Some(k) = self.prog.modules.iter().position(|m| &m.name == name) else { continue };
            for (n, v) in self.module_params(k) {
                out.insert(format!("{name}.{n}"), v);
            }
        }
        out
    }

    /// **An instance's numbers are read by its name** (`ball.r`): each numeric formal of every
    /// instance a body writes goes into the body's `vals` as `{instance}.{formal}` — the number
    /// it was given, or the drawing's unknown it became where it was left unbound or seeded
    /// (`bind`'s name) — so a dimension reads it as it reads a group's member, and an instance
    /// handed on as a group carries it (`bind`'s group arm copies `{actual}.…`).  Read before
    /// the body's statements are walked, as its `param`s are (P2: a dimension above the call
    /// reads it too).  Nothing is reported here: `bind` works the same arguments out again, by
    /// the same rules (`formals_of`, `typed`), and says what is wrong with them where the call is
    /// walked.
    pub(super) fn instance_numbers(
        &self,
        body: &[Stmt],
        vals: &mut BTreeMap<String, Aff>,
        scope: &Scope,
    ) {
        use crate::syntax::InstVal;
        for st in body {
            let StmtKind::Instance(inst) = &st.kind else { continue };
            let Ok(index) = self.prog.resolve_component(&inst.component.text, scope.module) else {
                continue;
            };
            let comp = &self.prog.components[index];
            let mut given: BTreeMap<&str, Aff> = BTreeMap::new();
            for (f, a) in formals_of(comp, inst) {
                let Some(f) = f.filter(|f| f.ty.number()) else { continue };
                let text = match &a.value {
                    InstVal::Expr(t) => t.clone(),
                    InstVal::Ref(r) => written(r),
                    InstVal::Hint(..) | InstVal::Group(_) => continue,
                };
                if let Some(v) = value_aff(&text, vals, self.units)
                    .and_then(|v| typed(v, f.ty, &f.name.text))
                    .ok()
                {
                    given.insert(&f.name.text, v);
                }
            }
            for f in comp.formals.iter().filter(|f| f.ty.number()) {
                let v = given.remove(f.name.text.as_str()).unwrap_or_else(|| {
                    free(formal_name(scope.prefix(), &inst.name.text, &f.name.text), f.ty)
                });
                vals.entry(format!("{}.{}", inst.name.text, f.name.text)).or_insert(v);
            }
        }
    }

    /// One value argument, worked out and bound under the formal's *declared* dimension.
    ///
    /// The formal declares, so it wins — but an argument that said what it was and disagreed is
    /// reported: `Tooth(a0: 30mm)` is a mistake, not a conversion.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn bind_value(
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
        match value_aff(text, vals, self.units) {
            Ok(a) => match typed(a, ty, &f.name.text) {
                Ok(v) => {
                    sub.insert(f.name.text.clone(), v);
                }
                Err(e) => self.err(Code::E103, span, e),
            },
            // a text a curve's variables leave no value to — kept, in the symbolic mode,
            // under the name the formal has inside the instance; a mistake, on the sheet
            Err(e) => {
                let abs = formal_name(scope.prefix(), inst, &f.name.text);
                if !self.keep_text(abs, text, vals, scope) {
                    self.err(Code::E103, span, format!("`{}`: {e}", f.name.text));
                }
            }
        }
    }
}

/// The formal each argument of a call lands on, in order: by its label, else by its place among
/// the unlabelled arguments — `None` where the component has no such formal.  `bind` and
/// `instance_numbers` read a call through this one rule, so `ball.r` read early is the number the
/// body is given.
fn formals_of<'c, 'i>(
    comp: &'c Component,
    inst: &'i crate::syntax::Instance,
) -> Vec<(Option<&'c crate::syntax::Formal>, &'i crate::syntax::InstArg)> {
    let mut positional = 0usize;
    inst.args
        .iter()
        .map(|a| {
            let f = match &a.label {
                Some(l) => comp.formals.iter().find(|f| f.name.text == l.text),
                None => {
                    positional += 1;
                    comp.formals.get(positional - 1)
                }
            };
            (f, a)
        })
        .collect()
}

/// The absolute name of a formal of the instance `inst` written under `prefix` (`leg.theta`): the
/// key its alias is filed under, the unknown a numeric one is where the call leaves it unbound,
/// and where a trace keeps an argument's text.
fn formal_name(prefix: &str, inst: &str, formal: &str) -> String {
    format!("{prefix}{inst}.{formal}")
}
