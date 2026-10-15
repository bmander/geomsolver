//! Source-order statement expansion, repetition, seed settling and emission.

use super::*;
use super::apply::Pass;
use super::sets::Made;
use std::ops::ControlFlow;

impl<'a> Walk<'a> {
    /// The view an enclosing instance is drawn `in`, put on one declaration its expansion
    /// makes (§6.7).  A datum or a curve is left alone — it has no points of its own to put
    /// there — and a declaration that already says which plane (a clause of its own, on a
    /// plane the component declares) may not be told twice.
    pub(super) fn stamp_scope_plane(&mut self, d: &mut Decl, scope: &Scope) {
        d.annotations.roles = d.annotations.roles.union(scope.stamps.roles);
        // the instance's classes, over the declaration's own: what the assembly says of an
        // instance is the later and stronger word, so a phantom's centreline is a phantom's
        for c in &scope.stamps.class.0 {
            if !d.class.has(c) {
                d.class.0.push(c.clone());
            }
        }
        let Some(p) = &scope.stamps.plane else { return };
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
            // a set's body walked again for a tangency states its relations' derivatives and
            // makes nothing a second time, and one walked for what it makes states nothing (§6.21)
            let relation = matches!(st.kind, StmtKind::Relation(_));
            match scope.pred.pass {
                Pass::Along(_) if !relation => continue,
                Pass::Made
                    if relation
                        || matches!(st.kind, StmtKind::ClaimOver(_) | StmtKind::Minimize(_) | StmtKind::Measured(_)) =>
                {
                    continue
                }
                _ => {}
            }
            if self.emitted() >= MAX_FLAT {
                self.err(
                    Code::E103,
                    st.span,
                    format!("more than {MAX_FLAT} statements once expanded"),
                );
                return;
            }
            match &st.kind {
                StmtKind::Chain(chain) => self.walk_chain(chain, st, scope, path, &prefix),
                StmtKind::Decl(d) => {
                    // a face family's station is drawn, hidden, before the solid it makes
                    if let Some(crate::syntax::Sweep::Family(f)) = &d.sweep {
                        if self.walk_family(f, st, scope, vals, path, depth).is_break() {
                            return;
                        }
                    }
                    self.walk_decl(d, st, scope, vals, path, &prefix)
                }
                StmtKind::Param(_) | StmtKind::Group(_) => {} // worked out above, before the walk
                StmtKind::Instance(inst) => {
                    if self.walk_instance(inst, st, scope, vals, path, depth).is_break() {
                        return;
                    }
                }
                StmtKind::Block(b) => self.walk_block(b, st, scope, vals, path, depth),
                // a constraint: its dimension is written in the component's own parameters, which
                // do not exist in the flat document, so they are worked out here
                // — and one written with a defined word is that word's body (§9.9)
                StmtKind::Relation(rel) => self.walk_relation(rel, st, scope, vals, path, depth),
                // a set written in place: nothing drawn, its uses expanded once every name is
                // known (§6.21)
                StmtKind::Set(set) => {
                    let abs = format!("{prefix}{}", set.name.text);
                    self.names.insert(abs.clone());
                    self.set_made(abs, &set.lit, scope, depth, Made { st, path, outer: scope });
                }
                // an energy: each integrand with the scope's numbers written in (#121)
                StmtKind::Minimize(m) => self.walk_minimize(m, st, scope, vals, path),
                // a bound on measurements: the scope's numbers written in, what it measures
                // resolved with the references (`rewrite`) (#183 F3, #190)
                StmtKind::Measured(m) => {
                    let mut m2 = m.clone();
                    for text in m2.lhs.numbers_mut().into_iter().chain(m2.rhs.numbers_mut()) {
                        let held = &self.held.names;
                        *text = super::values::substitute_in(text, vals, self.units, held);
                    }
                    self.emit(StmtKind::Measured(m2), st, scope, path);
                }
                // a claim over an interval: its numbers and its relations' worked out as above
                StmtKind::ClaimOver(c) => self.walk_claim_over(c, st, scope, vals, path, depth),
                // a gauge or an orientation: kept as written, resolved later
                other => self.emit(other.clone(), st, scope, path),
            }
        }
    }

    /// A named chain: its name made absolute under the scope, and stamped as its instances say.
    fn walk_chain(
        &mut self,
        chain: &crate::syntax::NamedChain,
        st: &Stmt,
        scope: &Scope,
        path: &[PathStep],
        prefix: &str,
    ) {
        let mut chain = chain.clone();
        let abs = format!("{prefix}{}", chain.name.key().text);
        self.names.insert(abs.clone());
        if chain.annotations.private { self.private_names.insert(abs.clone(), scope.owner.clone()); }
        chain.annotations.roles = chain.annotations.roles.union(scope.stamps.roles);
        chain.name = chain.name.prefixed(abs, scope.block.copies);
        if scope.stamps.anonymous {
            chain.name = crate::syntax::DeclName::Key(chain.name.key().clone());
        }
        self.emit(StmtKind::Chain(chain), st, scope, path);
    }

    /// A declaration: its name made absolute, its numbers worked out in the scope, and the
    /// twins, the probe and the set its region makes beside it.
    pub(super) fn walk_decl(
        &mut self,
        d: &Decl,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
        prefix: &str,
    ) {
        let abs = format!("{prefix}{}", d.name.key().text);
        self.names.insert(abs.clone());
        if d.annotations.private { self.private_names.insert(abs.clone(), scope.owner.clone()); }
        // a spline through a chain's corners waits for every chain (#243 L7, `through`)
        if d.through.is_some() {
            self.aside.fits.push(super::through::Fit {
                d: d.clone(),
                st: st.clone(),
                scope: scope.clone(),
                vals: vals.clone(),
                path: path.to_vec(),
                prefix: prefix.to_string(),
            });
            return;
        }
        let mut d2 = d.clone();
        d2.name = d.name.prefixed(abs, scope.block.copies);
        if scope.stamps.anonymous {
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
                return;
            }
            d2.computed = Some([
                (self.subst_sym(x, vals, scope), *xs),
                (self.subst_sym(y, vals, scope), *ys),
            ]);
            self.emit(StmtKind::Decl(d2), st, scope, path);
            return;
        }
        if let Some(c) = d2.curve.as_mut() {
            // the interval is written over the parameters in scope, like a number
            let second = c.second.as_mut().map(|(_, d)| d);
            let ends = [&mut c.domain.0, &mut c.domain.1].into_iter()
                .chain(second.into_iter().flat_map(|d| [&mut d.0, &mut d.1]));
            for t in ends {
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
                        point: ref_text(point),
                    });
                }
            }
        }
        // a face family: its interval worked out, its running number named, and the face it
        // sweeps read under the instance `walk_family` drew (#243 L1)
        if let Some(crate::syntax::Sweep::Family(f)) = d2.sweep.as_mut() {
            let key = format!("{}{}", scope.prefix(), f.instance.name.text);
            let held = format!("{key}.{}", f.swept.text);
            // over a chain, each leg's running number and the face it sweeps (#243 L4)
            let legs = self.held.chained.get(&key).cloned().unwrap_or_default();
            // refused where the instance was walked: no solid
            if !self.held.names.contains_key(&held) && legs.is_empty() {
                return;
            }
            let end = |t: &str| value_of(t, vals, self.units);
            if let (Ok(a), Ok(b)) = (end(&f.domain.0), end(&f.domain.1)) {
                f.domain = (crate::syntax::num(a), crate::syntax::num(b));
            }
            let instances: Vec<String> = if legs.is_empty() {
                vec![f.instance.name.text.clone()]
            } else {
                legs.iter().map(|(_, k)| format!("{}_{k}", f.instance.name.text)).collect()
            };
            f.held = Some(legs.first().map_or(held, |(h, _)| h.clone()));
            f.legs = legs;
            let faces = instances.into_iter().map(|text| {
                let mut face = f.face.clone();
                face.path.insert(0, crate::syntax::Seg::Field(face.root.clone()));
                face.root = Name { text, span: face.span };
                crate::syntax::Kid::Ref(face)
            }).collect();
            if let Some(slot) = d2.children.first_mut() {
                *slot = faces;
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
        let twins = self.twins_of(&d2, &d.name.key().text);
        let region: Vec<_> = match self.region_of(&mut d2, &d.name.key().text, scope) {
            Some(r) => vec![r],
            None => self.envelope_of(&mut d2, &d.name.key().text, scope),
        };
        self.emit(StmtKind::Decl(d2), st, scope, path);
        for twin in twins {
            self.emit(StmtKind::Decl(twin), st, scope, path);
        }
        // a region solid's probe, and its set applied to it; an envelope's or a meshing's
        for (probe, inside) in region {
            self.emit(StmtKind::Decl(probe), st, scope, path);
            self.emit(StmtKind::Relation(inside), st, scope, path);
        }
    }

    /// **A face family's station** (#243 L1): the instance written in place, drawn under a key
    /// the source cannot write (`#f12.`) with the running formal left unbound, so it is the free
    /// value `bind` makes of one (`#f12.s`) — and that value *held* (`Held`): the drawn station
    /// stands at the middle of the interval, the family solves it again at every other.
    fn walk_family(
        &mut self,
        f: &crate::syntax::FamilySpec,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
        depth: usize,
    ) -> ControlFlow<()> {
        let inst = &f.instance;
        let comp = match self.prog.resolve_component(&inst.component.text, scope.module) {
            Ok(i) => &self.prog.components[i],
            Err(m) => {
                self.err(Code::E103, inst.component.span, m);
                return ControlFlow::Continue(());
            }
        };
        let (swept, cname) = (&f.swept.text, &inst.component.text);
        let Some(formal) = comp.formals.iter().find(|fo| fo.name.text == *swept) else {
            self.err(Code::E103, f.swept.span, format!("`{cname}` has no formal `{swept}` to run"));
            return ControlFlow::Continue(());
        };
        if !matches!(formal.ty, Ty::Scalar | Ty::Length | Ty::Angle) {
            let m = format!("a face family runs a number, and `{swept}` is not one");
            self.err(Code::E103, f.swept.span, m);
            return ControlFlow::Continue(());
        }
        if inst.args.iter().any(|a| a.label.as_ref().is_some_and(|l| l.text == *swept)) {
            let m = format!("`{swept}` runs over the family's interval, so the call leaves it \
                             unbound");
            self.err(Code::E103, f.swept.span, m);
            return ControlFlow::Continue(());
        }
        let ends = [&f.domain.0, &f.domain.1].map(|t| {
            value_of(t, vals, self.units).and_then(|v| v.is_finite().then_some(v)
                .ok_or_else(|| "is not a finite number".to_string()))
        });
        let (a, b) = match ends {
            [Ok(a), Ok(b)] => (a, b),
            [Err(e), _] | [_, Err(e)] => {
                self.err(Code::E103, st.span, format!("the family's interval: {e}"));
                return ControlFlow::Continue(());
            }
        };
        if a >= b {
            let m = format!("a face family runs from its interval's start to a later end: `in ({}, \
                             {})` runs nowhere or backwards", f.domain.0, f.domain.1);
            self.err(Code::E080, st.span, m);
            return ControlFlow::Continue(());
        }
        // a path given a named chain: one leg per link the interval reaches (#243 L4), every
        // chain the call gives run in step (L7)
        let chains = self.chain_arguments(comp, inst, scope);
        if !chains.is_empty() {
            return self.walk_legs(f, inst, chains, (a, b), formal.ty, st, scope, vals, path,
                                  depth);
        }
        let held = format!("{}{}.{swept}", scope.prefix(), inst.name.text);
        self.held.names.insert(held, (formal.ty.dim(), 0.5 * (a + b)));
        self.walk_instance(inst, st, scope, vals, path, depth)
    }

    /// **A face family over a chain** (#243 L4, finding 6): the path formal given a named open
    /// chain, the station runs over it link by link — link `k` while `s` is in `(k, k + 1)` — as
    /// one family.  Each link the interval reaches is a **leg**: the instance drawn under its own
    /// key (`#f12_k`), the formal given that link, its own running number held at the middle of
    /// its part of the interval, and the family's formal read as `k` past it (`bind`), so a law
    /// in `s` reads the chain's `s` on every link; a contact pinned to `s` on the link reads the
    /// leg's own number, the fraction of that link's run (`settle_relation`).
    ///
    /// **Several chains run in step** (#243 L7): every `path` formal given a named open chain is
    /// given its link `k` on leg `k` — a guide and the rails of a loft through several sections,
    /// each a link a pair of neighbouring sections — so the chains must have as many links.
    #[allow(clippy::too_many_arguments)]
    fn walk_legs(
        &mut self,
        f: &crate::syntax::FamilySpec,
        inst: &crate::syntax::Instance,
        chains: Vec<(usize, Ref, Vec<(Ref, String)>)>,
        (a, b): (f64, f64),
        ty: Ty,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
        depth: usize,
    ) -> ControlFlow<()> {
        let swept = &f.swept.text;
        let n = chains[0].2.len();
        if let Some((_, r, other)) = chains.iter().find(|(_, _, l)| l.len() != n) {
            let (first, m) = (ref_text(&chains[0].1), other.len());
            let m = format!("a face family runs the chains it is given in step, link by link, and \
                             `{}` has {m} link{} where `{first}` has {n}", ref_text(r),
                            if m == 1 { "" } else { "s" });
            self.err(Code::E080, r.span, m);
            return ControlFlow::Continue(());
        }
        if ty != Ty::Scalar {
            let m = format!("a face family over a chain runs `{swept}` link by link, from 0 to \
                             the chain's {n}: it is a `Scalar`");
            self.err(Code::E103, f.swept.span, m);
            return ControlFlow::Continue(());
        }
        if a < 0.0 || b > n as f64 {
            let m = format!("the chain has {n} link{}, so a family over it runs within (0, {n}): \
                             link `k` while `{swept}` is in (k, k + 1)",
                            if n == 1 { "" } else { "s" });
            self.err(Code::E080, st.span, m);
            return ControlFlow::Continue(());
        }
        let key = format!("{}{}", scope.prefix(), inst.name.text);
        let mut legs = Vec::new();
        for k in 0..n {
            let (lo, hi) = (a.max(k as f64), b.min(k as f64 + 1.0));
            if hi - lo <= 1e-12 * (b - a) {
                continue;
            }
            let mut leg = inst.clone();
            leg.name.text = format!("{}_{k}", inst.name.text);
            let mut abs = Vec::with_capacity(chains.len());
            for (at, _, links) in &chains {
                let (link, full) = links[k].clone();
                leg.args[*at].value = crate::syntax::InstVal::Ref(link);
                abs.push(full);
            }
            let held = format!("{}{}.{swept}", scope.prefix(), leg.name.text);
            self.held.names.insert(held.clone(), (ty.dim(), 0.5 * (lo + hi) - k as f64));
            self.held.legs.insert(held.clone(), (k as f64, abs));
            legs.push((held, k as f64));
            self.walk_instance(&leg, st, scope, vals, path, depth)?;
        }
        self.held.chained.insert(key, legs);
        ControlFlow::Continue(())
    }

    /// The named chains a call gives its `path` formals (#243 L4, L7): each argument's index, the
    /// argument as written, and each link as a reference the call's scope reads with its absolute
    /// name.  A closed chain is no path a family can run along, its ends meeting; said here, and
    /// then no chain is run.
    fn chain_arguments(
        &mut self,
        comp: &Component,
        inst: &crate::syntax::Instance,
        scope: &Scope,
    ) -> Vec<(usize, Ref, Vec<(Ref, String)>)> {
        let mut out = Vec::new();
        for fo in comp.formals.iter().filter(|fo| fo.ty == Ty::Path) {
            match self.chain_argument(comp, inst, scope, fo) {
                Some(Some(c)) => out.push(c),
                Some(None) => return Vec::new(),
                None => {}
            }
        }
        out
    }

    /// The named chain a call gives the `path` formal `fo`, where it gives one: `Some(None)`
    /// where it is refused (said here).
    fn chain_argument(
        &mut self,
        comp: &Component,
        inst: &crate::syntax::Instance,
        scope: &Scope,
        fo: &crate::syntax::Formal,
    ) -> Option<Option<(usize, Ref, Vec<(Ref, String)>)>> {
        let alias = self.alias_table();
        {
            let Some(at) = super::bindings::arg_index(comp, inst, fo) else { return None };
            let crate::syntax::InstVal::Ref(r) = &inst.args[at].value else { return None };
            let Some((abs, rest)) = self.look(&alias).find(r, scope) else { return None };
            if !rest.is_empty() {
                return None;
            }
            let chain = self.out.iter().find_map(|(st, _, sc)| match &st.kind {
                StmtKind::Chain(c) if c.name.key().text == abs => Some((c.clone(), sc.clone())),
                _ => None,
            });
            let Some((chain, sc)) = chain else { return None };
            if chain.closed {
                let m = format!("`{}` is a closed chain, and a face family along it would meet \
                                 itself: run it over an open one", ref_text(r));
                self.err(Code::E080, r.span, m);
                return Some(None);
            }
            let mut links = Vec::new();
            for l in &chain.links {
                let full = self.look(&alias).find(l, &sc)
                    .map(|(t, rest)| std::iter::once(t).chain(rest).collect::<Vec<_>>().join("."));
                // the link by the name the call's scope reads it by
                let local = full.as_ref().and_then(|full| scope.prefixes.iter()
                    .find_map(|p| full.strip_prefix(p.as_str()).filter(|n| !n.is_empty())));
                let (Some(full), Some(local)) = (full.clone(), local) else {
                    let m = format!("`{}`'s link `{}` is not one the call can name",
                                    ref_text(r), ref_text(l));
                    self.err(Code::E103, r.span, m);
                    return Some(None);
                };
                links.push((Ref::dotted(local, r.span), full));
            }
            return Some(Some((at, r.clone(), links)));
        }
    }

    /// An instance: its arguments bound and its component's body walked under its prefix — or
    /// the walk stopped, past the instances' cap.
    fn walk_instance(
        &mut self,
        inst: &crate::syntax::Instance,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
        depth: usize,
    ) -> ControlFlow<()> {
        // a body of nothing but calls emits no statement, so the statements' cap
        // never stops a recursion branching at every level: the instances have one
        if self.instances.len() >= MAX_FLAT {
            self.once(Code::E103, st.span, format!("more than {MAX_FLAT} instances once expanded"));
            return ControlFlow::Break(());
        }
        let Some((comp, mut sub_vals, sides, key)) =
            self.bind_instance(inst, scope, vals, true)
        else {
            return ControlFlow::Continue(());
        };
        // the instance's own `in`, or the one already in force around it — both at
        // once is a plane given twice, which one statement may not do (§6.7)
        let in_plane = match (inst.membership.plane(), &scope.stamps.plane) {
            (Some(p), Some(_)) => {
                self.err(Code::E103, p.span, inst.membership.cause().to_string());
                scope.stamps.plane.clone()
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
            typed: scope.typed.clone(),
            prefixes: vec![key],
            closed: true,
            forbidden,
            groups: comp.formals.iter().filter(|f| matches!(f.ty, Ty::Group | Ty::Set)).map(|f| f.name.text.clone()).collect(),
            vals: sub_vals.clone(),
            // the sides this instance was given, and no others: a component reads a
            // side by the name of its own formal, as it reads every other argument
            sides,
            // and its body's calls name components from the file it was written in
            module: comp.module,
            block: InBlock {
                cyc: None,
                // a ring's copy reaches into the instances written in it
                ring: scope.block.ring.clone(),
                copies: scope.block.copies,
            },
            stamps: Stamps {
                roles: scope.stamps.roles.union(inst.annotations.roles),
                plane: in_plane,
                class: {
                    let mut c = scope.stamps.class.clone();
                    c.0.extend(inst.class.0.iter().cloned());
                    c
                },
                anonymous: scope.stamps.anonymous || inst.name.text.starts_with('#'),
            },
            pred: PredWalk { pass: Pass::Itself, written: None, held: false },
        };
        // a component reached again while it is still being expanded recurses
        // (§8), ended by a `when` over its numbers: given the numbers it was given
        // there, every choice comes out the same and nothing ends it, so that, and
        // nesting past the cap, is E003, said once at the call closing the cycle
        let same = self.instantiating.iter()
            .rposition(|(c, given)| std::ptr::eq(*c, comp) && *given == sub_vals);
        let deep = depth + 1 >= MAX_DEPTH
            && self.instantiating.iter().any(|(c, _)| std::ptr::eq(*c, comp));
        if same.is_some() || deep {
            let name = |c: &Component| c.name.as_ref().map_or(String::new(), |n| n.text.clone());
            let message = match same {
                None => format!("`{}` instantiates itself more than {MAX_DEPTH} deep: \
                                 its `when` does not end it soon enough", name(comp)),
                Some(k) => {
                    let mut through: Vec<String> = Vec::new();
                    for (c, _) in &self.instantiating[k + 1..] {
                        let n = format!("`{}`", name(c));
                        if !std::ptr::eq(*c, comp) && !through.contains(&n) {
                            through.push(n);
                        }
                    }
                    let through = match through.is_empty() {
                        true => String::new(),
                        false => format!(" through {}", through.join(", ")),
                    };
                    format!("`{}` instantiates itself{through} with the numbers it was \
                             given, so nothing ends it: a `when` over a number each \
                             call changes does", name(comp))
                }
            };
            self.once(Code::E003, st.span, message);
            return ControlFlow::Continue(());
        }
        let mut instance_path = path.to_vec();
        instance_path.push(PathStep::Instance(st.id));
        // an instance of a family of sets is the set, its formals bound (§6.21) —
        // and what its body makes without reading its point is the instance's,
        // made once here (F3)
        let own = match &comp.set {
            Some(lit) => {
                let name = sc.prefix().trim_end_matches('.').to_string();
                let (own, lit) = super::sets::instance_part(lit);
                let made = Made { st, path, outer: scope };
                self.set_made(name, &lit, &sc, depth + 1, made);
                own
            }
            None => Vec::new(),
        };
        self.instantiating.push((comp, sub_vals.clone()));
        // a point formal drawn in a plane and left out is the instance's own point there, made
        // by the call (§5 [0.95]) — in its formal's plane, whatever plane the instance is in
        let owned = self.owned_points(comp, inst, st, vals);
        if !owned.is_empty() {
            let mut at_call = sc.clone();
            at_call.stamps.plane = None;
            self.body(&owned, &at_call, &mut sub_vals, path, depth + 1);
        }
        self.body(&comp.body, &sc, &mut sub_vals, &instance_path, depth + 1);
        if !own.is_empty() {
            self.body(&own, &sc, &mut sub_vals, &instance_path, depth + 1);
        }
        self.instantiating.pop();
        ControlFlow::Continue(())
    }

    /// A block: a `when` made or not, one over a chain set aside until the chain is found, or
    /// its copies.
    fn walk_block(
        &mut self,
        b: &Block,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
        depth: usize,
    ) {
        // made once in this scope, or not at all (#210): no copy, no prefix, so what
        // it declares is named as if written beside it, and the numbers it defines
        // are its own
        if b.kind == BlockKind::When {
            let names = Asked { walk: self, scope, alias: std::cell::OnceCell::new() };
            match super::values::condition(&b.count, vals, scope, self.units, &names) {
                Ok(true) => {
                    let mut sub = vals.clone();
                    self.body(&b.body, scope, &mut sub, path, depth + 1);
                }
                Ok(false) => {}
                Err((code, message)) => self.once(code, b.head, message),
            }
            return;
        }
        if b.over.is_some() {
            // how many is the chain's, and the chain may be a name no walk has
            // reached yet (a forward reference, or one inside an instance written
            // further down): expanded once every name is known, where the block
            // itself stands in the output until then
            self.aside.blocks.push(Pending {
                st: st.clone(),
                scope: scope.clone(),
                vals: vals.clone(),
                path: path.to_vec(),
                depth,
            });
            self.out.push((st.clone(), path.to_vec(), scope.clone()));
            return;
        }
        // a count is a whole number: a computed one (`360deg / 15deg`) may carry
        // the arithmetic's last bit, but 2.5 is not rounded to anything (#115)
        let n = match value_of(&b.count, vals, self.units) {
            Ok(v) if v.is_finite() && v >= 0.0
                && (v - v.round()).abs() <= 1e-9 * v.max(1.0) => v.round() as usize,
            Ok(v) => {
                self.err(
                    Code::E103,
                    b.span,
                    format!("`{}` is {v}, which is not a count", b.count),
                );
                return;
            }
            Err(e) => {
                self.err(Code::E103, b.span, format!("`{}`: {e}", b.count));
                return;
            }
        };
        self.copies(b, st, scope, vals, path, depth, n, None);
    }

    /// A relation: a defined word's body applied, or its numbers worked out in the scope and
    /// stated — as its derivative where a tangency walks the body.
    fn walk_relation(
        &mut self,
        rel: &crate::syntax::Relation,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
        depth: usize,
    ) {
        // a step is the ordinates and levels it is short for, stated where it stands (#330)
        if self.walk_step(rel, st, scope, vals, path) {
            return;
        }
        // a word a file defines is its body, applied where the statement stands
        if self.apply_word(st, rel, path, scope, depth) {
            return;
        }
        let Some(mut r2) = self.settle_relation(rel, vals, scope) else { return };
        if let Some(twin) = scope.twin() {
            // a claim, a gauge and a bound state no row, so they have no
            // derivative
            // — `inside` or `outside` a plane among the bounds, lowered later
            let (gauge, bound) = match r2.form.written() {
                Some(w) => (crate::constraints::gauge_op(&w.word.text).is_some(),
                            w.states_bound()),
                None => (false, false),
            };
            if r2.claim || gauge || bound {
                return;
            }
            r2.along = Some(twin.clone());
        }
        self.emit(StmtKind::Relation(r2), st, scope, path);
    }

    /// An energy: each integrand with the scope's numbers written in (#121).
    fn walk_minimize(
        &mut self,
        m: &crate::syntax::Minimize,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
    ) {
        let mut m2 = m.clone();
        let mut ok = true;
        for t in &mut m2.terms {
            match super::values::settle_integrand(t, vals, self.units) {
                Ok(text) => t.body = text,
                Err(e) => {
                    self.err(Code::E101, t.body_span, e);
                    ok = false;
                }
            }
        }
        if ok {
            self.emit(StmtKind::Minimize(m2), st, scope, path);
        }
    }

    /// A claim over an interval: its numbers and its relations' worked out as a lone relation's.
    fn walk_claim_over(
        &mut self,
        c: &crate::syntax::ClaimOver,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
        depth: usize,
    ) {
        let mut c2 = c.clone();
        for a in [&mut c2.from, &mut c2.to] { self.settle_arg(a, vals, scope); }
        let mut body = Vec::with_capacity(c.body.len());
        for inner in &c.body {
            let StmtKind::Relation(rel) = &inner.kind else {
                body.push(inner.clone());
                continue;
            };
            // a word's relations are claimed as the statement is
            let from = self.out.len();
            if self.apply_word(inner, rel, path, scope, depth) {
                for (made, _, _) in self.out.drain(from..).collect::<Vec<_>>() {
                    match made.kind {
                        StmtKind::Relation(_) => body.push(made),
                        _ => self.once(Code::E103, inner.span, "a word in a swept claim \
                            states relations, and declares nothing"),
                    }
                }
                continue;
            }
            let Some(r2) = self.settle_relation(rel, vals, scope) else { continue };
            body.push(Stmt { kind: StmtKind::Relation(r2), ..inner.clone() });
        }
        c2.body = body;
        self.emit(StmtKind::ClaimOver(c2), st, scope, path);
    }

    /// A relation's numbers worked out against the parameters in scope, in either of its
    /// representations, and the enclosing instances' classes stamped over its own.
    pub(super) fn settle_relation(&mut self, rel: &crate::syntax::Relation, vals: &BTreeMap<String, Aff>, scope: &Scope)
        -> Option<crate::syntax::Relation> {
        let mut r2 = rel.clone();
        self.held.on_leg = self.leg_named(&r2, scope);
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
                    crate::syntax::OpArg::Bound { hi, grow, .. } => {
                        for (text, span) in hi.iter_mut().chain(grow.iter_mut()) {
                            self.settle_dim(text, *span, vals, scope)
                        }
                    }
                    crate::syntax::OpArg::Ent(_) => {}
                }
            }
        }
        for a in r2.form.canonical_args_mut().iter_mut().flatten() {
            self.settle_arg(a, vals, scope);
        }
        self.held.on_leg = None;
        // the enclosing instances' classes, over the statement's own: a ghosted part's
        // dimensions are the ghost's (`display: geometry`) as its lines are
        for c in &scope.stamps.class.0 {
            if !r2.class.has(c) {
                r2.class.0.push(c.clone());
            }
        }
        Some(r2)
    }

    /// The leg of a family over a chain whose link relation `r`, written in a leg's station,
    /// names among its operands (#243 L4) — a link of any chain the leg runs (L7): that leg's
    /// running number and the link's index.
    fn leg_named(&self, r: &crate::syntax::Relation, scope: &Scope) -> Option<(String, f64)> {
        if self.held.legs.is_empty() {
            return None;
        }
        let w = r.form.written()?;
        let prefix = scope.prefix();
        let (held, (k, links)) = self.held.legs.iter().find(|(h, _)| {
            prefix.starts_with(&crate::model::station_prefix(h))
        })?;
        let alias = self.alias_table();
        w.ops.iter().any(|op| self.look(&alias).find(op, scope).is_some_and(|(t, rest)| {
            links.contains(&std::iter::once(t).chain(rest).collect::<Vec<_>>().join("."))
        })).then(|| (held.clone(), *k))
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
        // starting value. Read it only in hints — where the unknown's own seed says it starts
        // (`lean: hint(20deg)`), else at zero; constraints keep the symbolic value. Traced
        // components retain their parameterized seeds.
        let provisional;
        let vals = if self.sym.is_none() && vals.values().any(|v| v.free.is_some()) {
            // a face family's running number starts where its drawn station stands (#243 L1),
            // and a text kept over it reads it there by its own name
            let held = |n: &String| self.held.names.get(n).map(|&(_, v)| v);
            provisional = vals.iter().map(|(name, value)| {
                let seed = value.free.as_ref()
                    .and_then(|n| self.unknowns.get(n).and_then(|d| d.seed).or_else(|| held(n)));
                (name.clone(), Aff::of_dim(value.c + seed.map_or(0.0, |s| value.m * s), value.dim))
            }).chain(self.held.names.iter().map(|(n, &(dim, v))| (n.clone(), Aff::of_dim(v, dim))))
                .collect();
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
            // how far a direction read off geometry is turned: a number over the parameters in
            // scope, read by the build as a place's `turn:` is
            if let Some((b, _)) = ks.dir.as_mut().and_then(|d| d.turn.as_mut()) {
                *b = if self.sym.is_some() {
                    self.subst_sym(b, vals, scope)
                } else {
                    super::values::substitute_in(b, vals, self.units, &self.held.names)
                };
            }
        }
        for (b, _) in d.seed_at.iter_mut().flat_map(|a| a.texts_mut()) {
            // a place's numbers are texts over the parameters in scope, read later — by the
            // trace compile off its variable table, or by the build off the geometry's seeds —
            // so the numbers in force are written in and the rest is left for that reader
            *b = if self.sym.is_some() {
                self.subst_sym(b, vals, scope)
            } else {
                super::values::substitute_in(b, vals, self.units, &self.held.names)
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
                *keep = Some(super::values::substitute_in(&t, vals, self.units, &self.held.names));
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
        let block_prefix = keys::block(scope.prefix(), st.id.0);
        // a ring: copies the representative's turns.  One inside another is refused (E022) and
        // expanded as the cycle it is besides, so what it makes is still there to be named; and
        // its index, read anywhere in its body, varies nothing (E015)
        let ring = match (b.kind, &b.about) {
            (BlockKind::Ring, Some(_)) if scope.block.ring.is_some() => {
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
                let written = (block_prefix.clone(), about.clone(), scope.clone(), n, st.span);
                self.rings.written.push(written);
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
                typed: scope.typed.clone(),
                prefixes: std::iter::once(format!("{block_prefix}{k}."))
                    .chain(scope.prefixes.iter().cloned())
                    .collect(),
                // `next` and `prev` mean something only where the copies close
                closed: scope.closed,
                forbidden: scope.forbidden.clone(),
                groups: scope.groups.clone(),
                vals: sub.clone(),
                sides: scope.sides.clone(),
                module: scope.module,
                block: InBlock {
                    cyc: b.kind.wraps().then(|| Cyc { prefix: block_prefix.clone(), k, n }),
                    ring: match &ring {
                        Some(prefix) => Some(super::Ring { prefix: prefix.clone(), turned: k > 0 }),
                        None => scope.block.ring.clone(),
                    },
                    // the prefix just built is the block's id, so every declaration
                    // below is a copy, however deep and through however many instances
                    copies: true,
                },
                stamps: scope.stamps.clone(),
                pred: PredWalk {
                    pass: scope.pred.pass.clone(),
                    written: None,
                    held: scope.pred.held,
                },
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
                        block: InBlock {
                            cyc: Some(Cyc { prefix: block_prefix.clone(), k, n }),
                            ..sc.block.clone()
                        },
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
        self.out.len() + self.aside.held
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
    pub(super) fn emit(&mut self, kind: StmtKind, st: &Stmt, scope: &Scope, path: &[PathStep]) {
        let stmt = Stmt { id: st.id, kind, span: st.span, chained: st.chained };
        self.out.push((stmt, path.to_vec(), scope.clone()));
    }
}

impl Walk<'_> {
    /// A point drawn in further planes (`point in P, G`, §6.7) is drawn in each by a **twin**: a
    /// point of its own there, keyed `{point}@{k}` and called by the point's name, seeded where
    /// the point is (`hint(at: point)`, read in space and seen in the twin's plane).  The
    /// elaborator ties each to the point in space (`Expansion::twins`).  `local` is the point's
    /// name as its scope reads it.
    fn twins_of(&mut self, d: &Decl, local: &str) -> Vec<Decl> {
        let point = d.name.key().text.clone();
        let at = Span::at(d.name.span().lo as usize);
        d.membership.also().iter().enumerate().map(|(k, plane)| {
            self.twin(&point, format!("{point}@{}", k + 1), at, Ref::new(local), plane.clone())
        }).collect()
    }

    /// The twin of `point` keyed `key`, drawn in `plane` and seeded at `seed` (the point as the
    /// scope it is declared in reads it), its name an empty span at `at` — recorded, so the
    /// elaborator ties it (`Expansion::twins`).
    fn twin(&mut self, point: &str, key: String, at: Span, seed: Ref, plane: Ref) -> Decl {
        self.names.insert(key.clone());
        self.twins.push((point.to_string(), key.clone()));
        let name = crate::syntax::DeclName::Key(Name { text: key, span: at });
        let mut twin = Decl::point(name, [0.0; 2], Some(crate::syntax::AtRef::at(seed)));
        twin.membership = crate::syntax::Membership::written_at(plane, at);
        twin
    }

    /// **A point drawn in a plane said to be on another is drawn in that one too** (§6.7,
    /// §6.21, #211): `gear.M coincident q`, where `gear.M` is drawn in `p` and `q` is a plane,
    /// is `gear.M` drawn in `q` by a twin, as `in p, q` would draw it — the statement made the
    /// twin's declaration in place of a row, so what is drawn in `q` reads the point there.  It
    /// is said here, once every declaration is walked, because the memberships the elaborator
    /// reads first must know the twin; a point that is drawn in no plane yet is left to
    /// `planes::incidences`, which draws it in the plane, and one already drawn in it to its row.
    /// The point is said first: `q coincident gear.M` puts the plane through the point, a row (a
    /// plane dropped through a point, `edit::add_datum`, may lie on its plane).  A claim, a
    /// derivative, a set's application and a ring's statement say a row as written.
    pub(super) fn drawn_incidences(&mut self, alias: &BTreeMap<String, String>) {
        use crate::model::EntKind;
        if self.sym.is_some() {
            return; // a traced body is solved for its point, never drawn
        }
        let units = self.units;
        let at = |r: &Ref, sc: &Scope, names: &BTreeSet<String>| {
            Lookup { names, alias, units }.find(r, sc)
                .and_then(|(abs, rest)| rest.is_empty().then_some(abs))
        };
        // every declaration walked, by its absolute name, and the standard datums, walked after
        // the document's own (`finish`) but planes and a point it may name like any other
        let mut decls: Vec<(String, &Decl, Scope)> =
            self.out.iter().filter_map(|(st, _, sc)| match &st.kind {
                StmtKind::Decl(d) => Some((d.name.key().text.clone(), d, sc.clone())),
                _ => None,
            }).collect();
        if let Some((comp, scope)) = self.datums() {
            decls.extend(comp.body.iter().filter_map(|st| match &st.kind {
                StmtKind::Decl(d) => Some((format!("std.{}", d.name.key().text), d, scope.clone())),
                _ => None,
            }));
        }
        let planes: BTreeSet<String> = decls.iter()
            .filter(|(_, d, _)| d.kind == EntKind::Plane)
            .map(|(key, ..)| key.clone())
            .collect();
        // each point its declaration draws in a plane, with every plane it is drawn in, first
        // first.  One its instance's `in` puts in a view stands in space to the component that
        // declares it, which says where it stands whatever view the instance is put in: its
        // statements read as a point in space's (`planes::incidences`)
        let mut drawn: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for (key, d, sc) in &decls {
            use crate::syntax::Source;
            if d.kind != EntKind::Point || d.membership.source() == Source::Instance {
                continue;
            }
            let Some(first) = d.membership.plane().and_then(|r| at(r, sc, &self.names)) else {
                continue;
            };
            let also = d.membership.also().iter().filter_map(|r| at(r, sc, &self.names));
            drawn.insert(key.clone(), std::iter::once(first).chain(also).collect());
        }
        for i in 0..self.out.len() {
            let (st, _, sc) = &self.out[i];
            let StmtKind::Relation(r) = &st.kind else { continue };
            if r.claim || r.along.is_some() || r.space || sc.block.ring.is_some() {
                continue;
            }
            let Some(w) = r.form.written() else { continue };
            let [rp, rq] = &w.ops[..] else { continue };
            if w.word.text != "coincident" || !w.args.is_empty() {
                continue;
            }
            // the point first: `P coincident p` puts the plane through the point, by a row
            let (Some(point), Some(plane)) = (at(rp, sc, &self.names), at(rq, sc, &self.names))
            else {
                continue;
            };
            let (rp, rq) = (rp.clone(), rq.clone());
            let Some(images) = drawn.get_mut(&point) else { continue };
            if !planes.contains(&plane) || images.contains(&plane) {
                continue;
            }
            // past the twins `in` made, and any a name already holds
            let mut k = images.len();
            while self.names.contains(&format!("{point}@{k}")) {
                k += 1;
            }
            images.push(plane);
            let span = Span::at(rp.span.lo as usize);
            let twin = self.twin(&point, format!("{point}@{k}"), span, rp, rq);
            self.out[i].0.kind = StmtKind::Decl(twin);
        }
    }
}

impl Walk<'_> {
    /// **A region as a solid** (§6.21): `blank := solid(R)`, its one operand a set, is the points
    /// inside it — the solid made `Sweep::Region` over a hidden point keyed `{solid}#probe`
    /// (held, in space), and `probe inside R` stated beside it, so the set's body is applied to
    /// the probe as any use applies it and the bounds it puts there are the solid's terms
    /// (`constrain`).
    /// `None` for any other declaration.
    fn region_of(&mut self, d: &mut Decl, local: &str, sc: &Scope)
        -> Option<(Decl, crate::syntax::Relation)> {
        use crate::syntax::Sweep;
        if d.kind != crate::model::EntKind::Solid || !matches!(d.sweep, Some(Sweep::Body)) {
            return None;
        }
        let set = self.set_operand(d, 0, sc)?;
        let at = Span::at(d.name.span().lo as usize);
        let key = keys::probe(&d.name.key().text);
        self.names.insert(key.clone());
        self.sets.probes.push(key.clone());
        let probe = keys::probe(local);
        d.children.clear();
        d.sweep = Some(Sweep::Region { probe: Ref::new(&probe) });
        Some((super::sets::probe(key, at), applied(&probe, set, "inside", at)))
    }

    /// **A set's envelope** (#283): `flank := envelope(S, under: m, …)`, its first operand a set,
    /// is the envelope of `S`'s surface — the operand made a hidden point keyed `{flank}#probe`
    /// (held, in space) and `probe coincident S` stated beside it, so the set's body is applied to
    /// the probe as any use applies it, and what it states there is the surface the envelope reads
    /// (a cone's angle, a cylinder's distance: `program::envelopes`), never a row.  A meshing's
    /// flank that is a set is read the same way, its two keyed `{contact}#first#probe` and
    /// `{contact}#second#probe`.  Empty for any other declaration.
    fn envelope_of(&mut self, d: &mut Decl, local: &str, sc: &Scope)
        -> Vec<(Decl, crate::syntax::Relation)> {
        let slots: &[(usize, &str)] = match d.kind {
            crate::model::EntKind::Envelope => &[(0, "")],
            crate::model::EntKind::Meshing => &[(0, "#first"), (1, "#second")],
            _ => return Vec::new(),
        };
        let mut made = Vec::new();
        for &(i, which) in slots {
            let Some(set) = self.set_operand(d, i, sc) else { continue };
            let at = Span::at(d.name.span().lo as usize);
            let key = keys::probe(&format!("{}{which}", d.name.key().text));
            self.names.insert(key.clone());
            let probe = keys::probe(&format!("{local}{which}"));
            d.children[i] = vec![Kid::Ref(Ref::new(&probe))];
            made.push((super::sets::probe(key, at), applied(&probe, set, "coincident", at)));
        }
        made
    }

    /// The set a declaration's `i`th operand names, where it names one.
    fn set_operand(&mut self, d: &Decl, i: usize, sc: &Scope) -> Option<Ref> {
        let [Kid::Ref(set)] = d.children.get(i).map(Vec::as_slice).unwrap_or(&[]) else {
            return None;
        };
        // a plain name declared in scope, as a family's instance is (`taken := solid(run)`), is
        // read off the names without the alias table, which a region asked for per solid would
        // build afresh each time (`Lookup::raw`'s order: each prefix, then the bare name)
        let plain = set.path.is_empty() && !matches!(set.root.text.as_str(), "next" | "prev");
        let candidates = sc.prefixes.iter().map(|p| format!("{p}{}", set.root.text))
            .chain((!sc.closed).then(|| set.root.text.clone()));
        let first = candidates.filter(|_| plain)
            .find(|abs| self.names.contains(abs) || self.aliases.iter().any(|(k, _, _)| k == abs));
        let found = match first {
            Some(abs) if self.names.contains(&abs) => Some((abs, Vec::new())),
            _ => {
                let alias = self.alias_table();
                self.look(&alias).raw(set, sc)
            }
        };
        let Some((abs, rest)) = found else { return None };
        if !rest.is_empty() || !self.sets.made.contains_key(&abs) {
            return None;
        }
        Some(set.clone())
    }
}

/// What a `when` asks of names that are not numbers (#267), answered by the walk as it stands
/// where the block is met: the aliases made so far — a face a copy runs over among them — read
/// once, and only if a condition asks.
struct Asked<'w, 'a> {
    walk: &'w Walk<'a>,
    scope: &'w Scope,
    alias: std::cell::OnceCell<BTreeMap<String, String>>,
}

impl Asked<'_, '_> {
    fn alias(&self) -> &BTreeMap<String, String> {
        self.alias.get_or_init(|| self.walk.alias_table())
    }
}

impl super::values::Names for Asked<'_, '_> {
    fn stands(&self, name: &str) -> Option<String> {
        self.walk.stands_for(&Ref::dotted(name, Span::default()), self.scope, self.alias())
    }

    fn faces(&self, name: &str) -> Option<Vec<String>> {
        self.walk.faces_named(&Ref::dotted(name, Span::default()), self.scope, self.alias())
    }

    fn running(&self, name: &str) -> bool {
        self.walk.held.names.contains_key(name)
    }
}

/// `probe word set`, stated by the walk at `at`: how a probe has its set applied to it.
fn applied(probe: &str, set: Ref, word: &str, at: Span) -> crate::syntax::Relation {
    let written = crate::syntax::Written {
        word: Name { text: word.to_string(), span: set.span },
        fixity: crate::constraints::Fixity::Infix,
        ops: vec![Ref::new(probe), set],
        args: Vec::new(),
        span: at,
    };
    crate::syntax::Relation::of(crate::syntax::RelationForm::Written(written))
}
