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
                self.err(
                    Code::E103,
                    a.span,
                    format!("`{}` has no such parameter", inst.component.text),
                );
                continue;
            };
            match (&f.ty, &a.value) {
                (Ty::Group, InstVal::Ref(r)) => {
                    let actual = written(r);
                    let key = format!("{prefix}{}.{}", inst.name.text, f.name.text);
                    self.aliases.push((key.clone(), r.clone(), scope.clone()));
                    self.group_bindings.push((key, a.span));
                    let start = format!("{actual}.");
                    for (name, value) in vals {
                        if let Some(member) = name.strip_prefix(&start) {
                            sub.insert(format!("{}.{member}", f.name.text), value.clone());
                        }
                    }
                }
                (Ty::Group, InstVal::Expr(t)) => self.err(Code::E103, a.span,
                    format!("`{}` wants a group or layout instance, not `{t}`", f.name.text)),
                (Ty::Ent(_), InstVal::Ref(r)) => {
                    // recorded unresolved; the resolve pass turns it into an absolute name in the
                    // *caller's* scope, which is what makes it an alias rather than a copy
                    self.aliases.push((
                        format!("{prefix}{}.{}", inst.name.text, f.name.text),
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
                        InstVal::Expr(t) => t.clone(),
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
        // A numeric formal the instance leaves unbound is an **unknown of the drawing**: the
        // language already makes a name nothing defines a free variable, and this is that
        // rule applied to a formal — a leg drawn with its crank angle unbound has a crank that
        // turns.  Named under the instance's own prefix (`leg.theta`), so two instances that
        // leave the same formal unbound have two unknowns and not one shared one — and inside
        // a traced component the name is no column of the curve, which is how a nested
        // instance's unbound formal is reported rather than captured by an outer one's.
        for f in &comp.formals {
            if f.ty == Ty::Group && !inst.args.iter().enumerate().any(|(i, a)|
                a.label.as_ref().map_or_else(|| comp.formals.get(i).is_some_and(|p| p.name.text == f.name.text), |l| l.text == f.name.text)) {
                self.err(Code::E103, inst.span, format!("missing group argument `{}`", f.name.text));
            }
            // a side left unbound is not an unknown of the drawing: the *statement* it reaches
            // says nothing about which side, which is the magnitude form and a solution set of
            // both — so it is left out of the table and the body writes no side at all
            if matches!(f.ty, Ty::Ent(_) | Ty::Side | Ty::Group) || sub.contains_key(&f.name.text) {
                continue;
            }
            let name = format!("{prefix}{}.{}", inst.name.text, f.name.text);
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
        let (body, uses) = (m.root.body.clone(), m.uses.clone());
        // the modules it uses first, under their paths, so its own params may read them
        let used = self.used_params(&uses);
        for (name, v) in &used {
            vals.insert(name.clone(), v.clone());
        }
        self.params(&body, &mut vals, &mut BTreeMap::new(), &scope);
        // and only its own go out: a file reaches another module's numbers through a `use` of
        // its own, so passing these on would only lengthen their names at every level
        vals.retain(|name, _| !used.contains_key(name));
        if let Some(slot) = self.module_vals.get_mut(k) {
            *slot = Some(vals.clone());
        }
        vals
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
        let want = ty.dim();
        match value_aff(text, vals, self.units) {
            Ok(a) => match a.dim.require(want, &f.name.text) {
                Ok(()) => {
                    sub.insert(f.name.text.clone(), a.as_dim(want));
                }
                Err(e) => self.err(Code::E103, span, e),
            },
            // a text a curve's variables leave no value to — kept, in the symbolic mode,
            // under the name the formal has inside the instance; a mistake, on the sheet
            Err(e) => {
                let abs = format!("{}{inst}.{}", scope.prefix(), f.name.text);
                if !self.keep_text(abs, text, vals, scope) {
                    self.err(Code::E103, span, format!("`{}`: {e}", f.name.text));
                }
            }
        }
    }
}
